import asyncio
import json
import logging
import math
import re

from multi_agent.llm_client import (
    call_llm, wants_tool_use, is_done, get_tool_calls,
    get_text_content, assistant_message, tool_result_message,
)
from multi_agent.mcp_client import MCPClient
from pathlib import Path

logger = logging.getLogger(__name__)

_SKILL_FILE = Path(__file__).parent / "skills" / "pick.md"

PICK_TOOLS = {
    "perception__segment_objects",
    "perception__get_topdown_grasp_pose",
    "moveit__plan_and_execute",
    "moveit__get_current_pose",
    "moveit__clear_planning_scene",
    "ros__call_service",
    "ros__send_action_goal",
    "ros__subscribe_once",
}

CREEP_TOOL = {
    "type": "function",
    "function": {
        "name": "creep_closer",
        "description": (
            "Drive the base to a 1.05m standoff facing the grasp target. "
            "Use ONLY when get_topdown_grasp_pose returns x > 1.10m. "
            "Computes the approach pose from the cached grasp (x, y) in "
            "base_footprint + live nav2 pose, then dispatches "
            "nav2.navigate_to_pose (long hop) or DriveOnHeading (short hop). "
            "After the move: clears the octomap, re-segments on the arm "
            "camera, and recomputes the grasp. Returns new segmentation + "
            "grasp output, prefixed with STATUS: TARGET_VISIBLE on success "
            "or STATUS: TARGET_LOST if post-drive arm-cam re-seg failed. "
            "Call repeatedly while grasp_x keeps decreasing (>=5cm per call). "
            "Cap: up to 3 creeps total. If a creep does NOT decrease grasp_x "
            "by >=5cm vs the previous value, the robot is stuck — report "
            "FAILURE. If after 3 creeps grasp_x still > 1.10m but progress "
            "was steady, attempt the grasp anyway with the latest pose."
        ),
        "parameters": {
            "type": "object",
            "properties": {
                "object_name": {
                    "type": "string",
                    "description": "Same prompt passed to segment_objects.",
                },
                "current_grasp_x": {
                    "type": "number",
                    "description": "x coordinate of the latest grasp pose in base_footprint (the value > 0.85m that triggered the creep).",
                },
            },
            "required": ["object_name", "current_grasp_x"],
        },
    },
}


def _filter_tools(all_tools: list[dict]) -> list[dict]:
    return [t for t in all_tools if t["function"]["name"] in PICK_TOOLS]


def _parse_seg_status(seg_raw) -> str:
    """Extract 'status' field from segment_objects response (handles str|dict)."""
    if not isinstance(seg_raw, str):
        seg_raw = str(seg_raw)
    try:
        return json.loads(seg_raw).get("status", "UNKNOWN")
    except (json.JSONDecodeError, AttributeError):
        return "UNKNOWN"


def _parse_robot_pose(raw) -> tuple[float | None, float | None, float | None]:
    """Extract (x, y, yaw) from nav2__get_robot_pose response.

    Returns (None, None, None) on parse failure. Uses nav2__get_robot_pose
    (live TF) instead of /amcl_pose topic — AMCL only publishes
    intermittently and the latched value can be minutes stale.
    """
    if not isinstance(raw, str):
        raw = str(raw)
    try:
        d = json.loads(raw)
        # nav2-mcp wraps the body as {"result": "<JSON string>"}
        body = d.get("result", d)
        if isinstance(body, str):
            body = json.loads(body)
        pos = body["position"]
        ori = body["orientation"]
        rx = float(pos["x"])
        ry = float(pos["y"])
        ryaw = float(ori["yaw"])
        return rx, ry, ryaw
    except (json.JSONDecodeError, KeyError, TypeError, ValueError):
        return None, None, None


async def _creep_closer(
    mcp: MCPClient,
    object_name: str,
    current_grasp_x: float,
) -> str:
    """Drive to a computed approach pose using nav2.navigate_to_pose.

    Replaces the previous DriveOnHeading approach (forward-only, blew
    past laterally-offset targets). Reads cached grasp pose for full
    (x, y) in base_footprint, projects to map frame using fresh
    /amcl_pose, computes a STANDOFF_M-back approach pose facing the
    target, and dispatches navigate_to_pose. nav2 handles spin + drive
    + obstacle avoidance.

    If post-nav arm-cam re-segmentation returns NO_OBJECTS_FOUND,
    spin-search up to SPIN_MAX × SPIN_ANGLE before reporting failure.
    """
    # Larger standoff than the reach limit so the target stays in arm-cam
    # FOV after the hop. At 0.85m the robot is RIGHT ON TOP of a small
    # floor object and look_forward arm cam frames it below the FOV
    # bottom — segmentation then loses the target and spin-search can't
    # recover (object too close to see). 1.05m keeps it visible while
    # still inside UR5's 1.10m reach envelope.
    STANDOFF_M = 1.05
    REACH_M = 1.10  # UR5 effective reach (matches pick.md)
    NAV_WALL_TIMEOUT = 90.0  # generous cap for low-RTF sim

    # Re-fetch cached grasp pose for both x AND y in base_footprint.
    # The LLM only passes grasp_x via current_grasp_x; we need y for
    # the approach-pose computation. Cache is intact — this is just a
    # TF transform on the existing pointcloud, no re-segmentation.
    # We also use this LIVE x to decide whether to move at all (the LLM
    # may have called us with a stale current_grasp_x value).
    cached_raw = await mcp.call_tool_prefixed(
        "perception__get_topdown_grasp_pose",
        {"object_name": object_name},
    )
    try:
        cached = json.loads(cached_raw) if isinstance(cached_raw, str) else cached_raw
        target_x_base = float(cached["centroid_base_frame"]["x"])
        target_y_base = float(cached["centroid_base_frame"]["y"])
    except (json.JSONDecodeError, KeyError, TypeError) as e:
        return (
            f"creep_closer: failed to read cached grasp pose ({e}). "
            f"Cannot compute approach pose. Call segment_objects + "
            f"get_topdown_grasp_pose first."
        )

    # Refuse to move if target is "close enough" (live cache check, NOT
    # the LLM-passed current_grasp_x which can be stale across calls).
    # Threshold = STANDOFF_M + 0.20m: if we're already at or just past
    # the standoff, attempting another creep risks driving PAST the
    # target (each DriveOnHeading hop is approximate; another hop with
    # only 0.15m to gain easily over-shoots → cube falls below arm-cam
    # FOV → spin-search recovers wrong object → wrong grasp).
    DONT_RECREEP_M = STANDOFF_M + 0.20
    if target_x_base <= DONT_RECREEP_M:
        return (
            f"creep_closer: target already close enough "
            f"(live grasp_x={target_x_base:.2f}m ≤ {DONT_RECREEP_M:.2f}m); "
            f"NOT moving. The LLM-passed current_grasp_x={current_grasp_x:.2f}m "
            f"may be stale. Attempt the grasp at the current pose; if MoveIt "
            f"fails to plan, the target is genuinely out of reach from here — "
            f"report FAILURE rather than calling creep again."
        )

    # Get current robot pose via nav2__get_robot_pose (live TF, fresh).
    # Avoid /amcl_pose subscribe — AMCL only publishes intermittently and
    # the latched value can be minutes stale.
    pose_raw = await mcp.call_tool_prefixed("nav2__get_robot_pose", {})
    rx, ry, ryaw = _parse_robot_pose(pose_raw)
    if rx is None:
        return (
            f"creep_closer: failed to parse nav2__get_robot_pose. "
            f"Raw: {str(pose_raw)[:300]}"
        )

    # Project target from base_footprint to map frame
    cos_y, sin_y = math.cos(ryaw), math.sin(ryaw)
    target_map_x = rx + cos_y * target_x_base - sin_y * target_y_base
    target_map_y = ry + sin_y * target_x_base + cos_y * target_y_base

    # Approach pose: STANDOFF_M back from target along robot→target vector
    dx = target_map_x - rx
    dy = target_map_y - ry
    dist = math.hypot(dx, dy)
    if dist < 0.01:
        return "creep_closer: target distance ~0; nothing to do."
    ux, uy = dx / dist, dy / dist
    approach_x = target_map_x - STANDOFF_M * ux
    approach_y = target_map_y - STANDOFF_M * uy
    approach_yaw = math.atan2(dy, dx)  # face the target

    # Hop distance the robot needs to travel
    hop_dist = dist - STANDOFF_M

    logger.info(
        f"  [creep] target_base=({target_x_base:.2f},{target_y_base:.2f}) "
        f"target_map=({target_map_x:.2f},{target_map_y:.2f}) "
        f"approach=({approach_x:.2f},{approach_y:.2f},yaw={approach_yaw:.2f}) "
        f"hop={hop_dist:.2f}m"
    )

    NAV_MIN_HOP_DIST = 0.40  # below nav2 xy_goal_tolerance + margin

    nav_note = ""
    if hop_dist < NAV_MIN_HOP_DIST:
        # Short hop: navigate_to_pose silently no-ops on goals < tolerance.
        # Use DriveOnHeading (behavior server, bypasses planner + tolerance).
        # First spin to face target if lateral offset is significant.
        bearing = math.atan2(target_y_base, target_x_base)
        if abs(bearing) > math.radians(8):
            try:
                await asyncio.wait_for(
                    mcp.call_tool_prefixed(
                        "nav2__spin_robot", {"angle": bearing}
                    ),
                    timeout=20.0,
                )
                await asyncio.sleep(1.0)
            except asyncio.TimeoutError:
                nav_note = "WARNING: spin to face target timed out. "

        drive_dist = max(hop_dist, 0.20)  # ensure visible movement
        logger.info(
            f"  [creep] short-hop DriveOnHeading {drive_dist:.2f}m "
            f"(bearing was {math.degrees(bearing):.1f}°)"
        )
        try:
            await asyncio.wait_for(
                mcp.call_tool_prefixed(
                    "ros__send_action_goal",
                    {
                        "action_name": "/drive_on_heading",
                        "action_type": "nav2_msgs/action/DriveOnHeading",
                        "goal": {
                            "target": {"x": drive_dist, "y": 0.0, "z": 0.0},
                            "speed": 0.3,
                            "time_allowance": {
                                "sec": int(drive_dist * 4 + 5),
                                "nanosec": 0,
                            },
                        },
                        "timeout": 30.0,
                    },
                ),
                timeout=35.0,
            )
        except asyncio.TimeoutError:
            nav_note += (
                f"WARNING: DriveOnHeading short-hop timed out after 35s. "
            )
    else:
        # Longer hop: navigate_to_pose handles spin + drive + obstacle avoidance.
        try:
            await asyncio.wait_for(
                mcp.call_tool_prefixed(
                    "nav2__navigate_to_pose",
                    {"x": approach_x, "y": approach_y, "yaw": approach_yaw},
                ),
                timeout=NAV_WALL_TIMEOUT,
            )
        except asyncio.TimeoutError:
            logger.warning(
                f"  [creep] navigate_to_pose timed out after {NAV_WALL_TIMEOUT:.0f}s"
            )
            nav_note = (
                f"WARNING: navigate_to_pose timed out after "
                f"{NAV_WALL_TIMEOUT:.0f}s. "
            )

    # Settle: nav2 reports goal-reached before robot fully decelerates AND
    # before arm-cam segmentation publishes a frame from the new pose.
    await asyncio.sleep(1.5)

    # Clear octomap, re-segment on arm cam.
    await mcp.call_tool_prefixed(
        "ros__call_service",
        {
            "service_name": "/clear_octomap",
            "service_type": "std_srvs/srv/Empty",
            "request": {},
        },
    )
    seg_raw = await mcp.call_tool_prefixed(
        "perception__segment_objects",
        {"prompt": object_name, "camera": "arm"},
    )

    # Spin-search recovery REMOVED 2026-05-02. The 6×60° loop was rotating
    # the robot a full 360° back to its starting orientation AND latching
    # onto wrong objects. If post-drive re-segment fails, return cleanly
    # with a TARGET_LOST status so the pick agent can call look() and
    # reason about the image (per pick.md rule).
    if _parse_seg_status(seg_raw) != "SUCCESS":
        return (
            f"creep_closer: navigated to approach pose "
            f"({approach_x:.2f}, {approach_y:.2f}, yaw={approach_yaw:.2f}), "
            f"BUT post-drive arm-cam re-segmentation returned "
            f"{_parse_seg_status(seg_raw)} for '{object_name}'. {nav_note}"
            f"STATUS: TARGET_LOST. "
            f"Recommended: call perception__look(camera='arm') to inspect "
            f"the current view, then either retry perception__segment_objects "
            f"with a refined prompt (geometric descriptor, e.g. "
            f"'small white cube on wooden floor') or report FAILURE if the "
            f"target is genuinely not visible."
        )

    # Recompute grasp pose from fresh pointcloud
    grasp = await mcp.call_tool_prefixed(
        "perception__get_topdown_grasp_pose",
        {"object_name": object_name},
    )
    return (
        f"creep_closer: STATUS: TARGET_VISIBLE. Navigated to approach pose "
        f"({approach_x:.2f}, {approach_y:.2f}, yaw={approach_yaw:.2f}), "
        f"re-segmented on arm camera, recomputed grasp.\n"
        f"{nav_note}"
        f"--- segmentation ---\n{seg_raw}\n--- grasp ---\n{grasp}"
    )


async def execute_pick(
    mcp: MCPClient,
    object_name: str,
    model: str = None,
    max_tool_calls: int = 30,
) -> dict:
    """Run a pick agent to grasp an object.

    The robot must already be positioned roughly in front of the object
    (navigator handoff). The agent may invoke the `creep_closer` helper
    up to twice if the initial grasp is just out of arm reach.

    Args:
        mcp: Connected MCPClient instance.
        object_name: Name of the object to pick (e.g. "red ball").
        model: LiteLLM model string. Defaults to LLM_MODEL env var.
        max_tool_calls: Safety cap on tool calls to prevent runaway.

    Returns:
        {"success": bool, "reason": str, "tool_calls_used": int}
    """
    all_tools = mcp.get_tools()
    tools = _filter_tools(all_tools) + [CREEP_TOOL]

    system_prompt = _SKILL_FILE.read_text()
    user_message = f"Pick up the object '{object_name}'."

    messages = [
        {"role": "system", "content": system_prompt},
        {"role": "user", "content": user_message},
    ]
    tool_call_count = 0

    logger.info(f"Pick started: '{object_name}'")

    while tool_call_count < max_tool_calls:
        response = call_llm(messages=messages, tools=tools, model=model)

        if wants_tool_use(response):
            messages.append(assistant_message(response))

            tool_results = []
            for tc_id, tc_name, tc_args in get_tool_calls(response):
                tool_call_count += 1
                logger.info(
                    f"  [{tool_call_count}] {tc_name}({json.dumps(tc_args)[:200]})"
                )

                try:
                    if tc_name == "creep_closer":
                        result = await _creep_closer(mcp, **tc_args)
                    else:
                        result = await mcp.call_tool_prefixed(tc_name, tc_args)
                    if len(result) > 3000:
                        result = result[:3000] + "\n... (truncated)"
                    tool_results.append(tool_result_message(tc_id, result))
                except Exception as e:
                    logger.error(f"  Tool error: {e}")
                    tool_results.append(
                        tool_result_message(tc_id, f"Error calling tool: {e}")
                    )

            messages.extend(tool_results)

        elif is_done(response):
            final_text = get_text_content(response)
            logger.info(f"Pick finished: {final_text[:200]}")

            # Find last standalone SUCCESS/FAILURE; whichever appears later wins
            s_matches = list(re.finditer(r'\bSUCCESS\b', final_text, re.IGNORECASE))
            f_matches = list(re.finditer(r'\bFAILURE\b', final_text, re.IGNORECASE))
            last_s = s_matches[-1].start() if s_matches else -1
            last_f = f_matches[-1].start() if f_matches else -1
            success = last_s > last_f
            return {
                "success": success,
                "reason": final_text,
                "tool_calls_used": tool_call_count,
            }
        else:
            stop = response.choices[0].finish_reason
            logger.warning(f"Unexpected finish_reason: {stop}")
            return {
                "success": False,
                "reason": f"Unexpected stop: {stop}",
                "tool_calls_used": tool_call_count,
            }

    logger.warning(f"Pick hit max tool calls ({max_tool_calls})")
    return {
        "success": False,
        "reason": f"Exceeded maximum tool calls ({max_tool_calls})",
        "tool_calls_used": tool_call_count,
    }
