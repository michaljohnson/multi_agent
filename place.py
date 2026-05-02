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

_SKILL_FILE = Path(__file__).parent / "skills" / "place.md"

PLACE_TOOLS = {
    "perception__segment_objects",
    "perception__get_topdown_placing_pose",
    "moveit__plan_and_execute",
    "moveit__get_current_pose",
    "moveit__clear_planning_scene",
    "ros__call_service",
    "ros__send_action_goal",
    "ros__publish_once",
}

CREEP_TOOL = {
    "type": "function",
    "function": {
        "name": "creep_closer",
        "description": (
            "Drive the base forward to bring the stage-1 coarse target "
            "into UR5 top-down reach. Use ONLY when stage 1's "
            "`get_topdown_placing_pose` returns a surface_centroid with "
            "sqrt(x^2 + y^2) > 0.65m. Reads /amcl_pose, drives nav2 "
            "forward by (current_dist - 0.55)m, clears the octomap, "
            "re-segments the target on the ARM camera (arm in "
            "look_down_high — same pose as stage 1), and recomputes the "
            "coarse drop pose. Returns the new segmentation + drop pose. "
            "Call repeatedly while distance keeps decreasing (>=5cm per "
            "call). Up to 5 creeps total — if reach is achieved earlier, "
            "stop. If two consecutive creeps fail to decrease distance "
            "by >=5cm, the robot is stuck; report FAILURE rather than "
            "creeping more. If after 5 creeps distance is still > 0.65m "
            "but progress was being made, proceed to stage 2 anyway "
            "(MoveIt may still plan; better than aborting)."
        ),
        "parameters": {
            "type": "object",
            "properties": {
                "target_container": {
                    "type": "string",
                    "description": "Same prompt passed to segment_objects.",
                },
                "current_target_x": {
                    "type": "number",
                    "description": "x of the latest surface_centroid in base_footprint.",
                },
                "current_target_y": {
                    "type": "number",
                    "description": "y of the latest surface_centroid in base_footprint.",
                },
                "top_clearance_m": {
                    "type": "number",
                    "description": (
                        "Clearance for the recomputed drop pose: 0.35 for "
                        "containers, 0.20 for surfaces."
                    ),
                },
            },
            "required": [
                "target_container",
                "current_target_x",
                "current_target_y",
                "top_clearance_m",
            ],
        },
    },
}


def _filter_tools(all_tools: list[dict]) -> list[dict]:
    return [t for t in all_tools if t["function"]["name"] in PLACE_TOOLS]


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
        body = d.get("result", d)
        if isinstance(body, str):
            body = json.loads(body)
        pos = body["position"]
        ori = body["orientation"]
        return float(pos["x"]), float(pos["y"]), float(ori["yaw"])
    except (json.JSONDecodeError, KeyError, TypeError, ValueError):
        return None, None, None


def _parse_seg_status(seg_raw) -> str:
    if not isinstance(seg_raw, str):
        seg_raw = str(seg_raw)
    try:
        return json.loads(seg_raw).get("status", "UNKNOWN")
    except (json.JSONDecodeError, AttributeError):
        return "UNKNOWN"


async def _creep_closer(
    mcp: MCPClient,
    target_container: str,
    current_target_x: float,
    current_target_y: float,
    top_clearance_m: float,
) -> str:
    """Drive to a computed approach pose. Mirrors pick.py _creep_closer:

    - Long hop (>= 0.40m): navigate_to_pose to (target_map - STANDOFF *
      unit_to_target). nav2 handles spin + drive + obstacle avoidance.
    - Short hop (< 0.40m): DriveOnHeading (bypasses nav2 xy_goal_tolerance
      silent no-op for tiny goals).
    - Refuses if cached target is already close (<= STANDOFF + 0.20m) so
      a second creep call from a stale-arg LLM doesn't drive past target.
    - After move: clear octomap, re-segment on arm cam, recompute drop pose.
    - If re-segment loses target, spin-search up to 6×60° to recover.
    """
    # Larger standoff than pick because (a) place targets are typically
    # larger objects (bins, tables) so getting too close risks bumper
    # collision, and (b) stage-2 (arm cam from above) refines the drop
    # pose precisely — we don't need pinpoint accuracy from creep.
    # Bumped 0.55 → 0.80m on 2026-05-02 after orchestrator e2e demo
    # showed robot driving too close to trash bin.
    STANDOFF_M = 0.80
    DONT_RECREEP_M = STANDOFF_M + 0.25  # 1.05m — refuse extra creep if
    SPIN_MAX = 6
    SPIN_ANGLE = 1.047
    NAV_WALL_TIMEOUT = 90.0
    NAV_MIN_HOP_DIST = 0.40

    # Use live cached drop pose to decide whether to move (LLM-passed
    # x,y may be stale across calls).
    cached_raw = await mcp.call_tool_prefixed(
        "perception__get_topdown_placing_pose",
        {"object_name": target_container, "top_clearance_m": top_clearance_m},
    )
    try:
        cached = json.loads(cached_raw) if isinstance(cached_raw, str) else cached_raw
        live_x = float(cached["surface_centroid"]["x"])
        live_y = float(cached["surface_centroid"]["y"])
    except (json.JSONDecodeError, KeyError, TypeError):
        # Fallback to LLM-passed values
        live_x, live_y = current_target_x, current_target_y

    live_dist = math.hypot(live_x, live_y)
    if live_dist <= DONT_RECREEP_M:
        return (
            f"creep_closer: target already close enough "
            f"(live dist={live_dist:.2f}m ≤ {DONT_RECREEP_M:.2f}m); NOT moving. "
            f"Proceed with stage 2 / drop. Calling creep again would "
            f"risk over-shooting the target."
        )

    # Get fresh robot pose in map frame
    pose_raw = await mcp.call_tool_prefixed("nav2__get_robot_pose", {})
    rx, ry, ryaw = _parse_robot_pose(pose_raw)
    if rx is None:
        return (
            f"creep_closer: failed to parse nav2__get_robot_pose. "
            f"Raw: {str(pose_raw)[:300]}"
        )

    # Project target from base_footprint to map
    cos_y, sin_y = math.cos(ryaw), math.sin(ryaw)
    target_map_x = rx + cos_y * live_x - sin_y * live_y
    target_map_y = ry + sin_y * live_x + cos_y * live_y

    dx, dy = target_map_x - rx, target_map_y - ry
    dist_to_target = math.hypot(dx, dy)
    if dist_to_target < 0.01:
        return "creep_closer: target distance ~0; nothing to do."
    ux, uy = dx / dist_to_target, dy / dist_to_target
    approach_x = target_map_x - STANDOFF_M * ux
    approach_y = target_map_y - STANDOFF_M * uy
    approach_yaw = math.atan2(dy, dx)

    hop_dist = dist_to_target - STANDOFF_M

    logger.info(
        f"  [creep] target_base=({live_x:.2f},{live_y:.2f}) "
        f"target_map=({target_map_x:.2f},{target_map_y:.2f}) "
        f"approach=({approach_x:.2f},{approach_y:.2f},yaw={approach_yaw:.2f}) "
        f"hop={hop_dist:.2f}m"
    )

    nav_note = ""
    if hop_dist < NAV_MIN_HOP_DIST:
        # Short hop: DriveOnHeading (bypasses nav2 tolerance).
        bearing = math.atan2(live_y, live_x)
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

        drive_dist = max(hop_dist, 0.20)
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
            nav_note += "WARNING: DriveOnHeading short-hop timed out. "
    else:
        # Longer hop: navigate_to_pose handles spin + drive + obstacles.
        try:
            await asyncio.wait_for(
                mcp.call_tool_prefixed(
                    "nav2__navigate_to_pose",
                    {"x": approach_x, "y": approach_y, "yaw": approach_yaw},
                ),
                timeout=NAV_WALL_TIMEOUT,
            )
        except asyncio.TimeoutError:
            nav_note = (
                f"WARNING: navigate_to_pose timed out after "
                f"{NAV_WALL_TIMEOUT:.0f}s. "
            )

    await asyncio.sleep(1.5)
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
        {"prompt": target_container, "camera": "arm"},
    )

    # Spin-search recovery if target lost
    spin_note = ""
    if _parse_seg_status(seg_raw) != "SUCCESS":
        logger.warning(
            f"  [creep] target lost after nav; spin-searching arm cam"
        )
        recovered = False
        for i in range(SPIN_MAX):
            await mcp.call_tool_prefixed(
                "nav2__spin_robot", {"angle": SPIN_ANGLE}
            )
            await asyncio.sleep(1.5)
            retry_raw = await mcp.call_tool_prefixed(
                "perception__segment_objects",
                {"prompt": target_container, "camera": "arm"},
            )
            if _parse_seg_status(retry_raw) == "SUCCESS":
                logger.info(
                    f"  [creep] recovered target after {i+1} spin(s)"
                )
                seg_raw = retry_raw
                spin_note = (
                    f"Spin-search recovered target after {i+1} 60-deg spin(s). "
                )
                recovered = True
                break
        if not recovered:
            return (
                f"creep_closer: navigated to ~{STANDOFF_M}m approach pose, "
                f"but target lost AND spin-search failed "
                f"({SPIN_MAX} × 60-deg tried). {nav_note}Report FAILURE."
            )

    drop = await mcp.call_tool_prefixed(
        "perception__get_topdown_placing_pose",
        {
            "object_name": target_container,
            "top_clearance_m": top_clearance_m,
        },
    )
    return (
        f"creep_closer: navigated to approach pose "
        f"({approach_x:.2f}, {approach_y:.2f}, yaw={approach_yaw:.2f}), "
        f"re-segmented on arm camera, recomputed drop pose.\n"
        f"{nav_note}{spin_note}"
        f"--- segmentation ---\n{seg_raw}\n--- drop pose ---\n{drop}"
    )


async def _verify_object_placed(
    mcp: MCPClient, object_name: str
) -> tuple[bool, str]:
    """Ground-truth check: after place agent reports success, verify the
    held object is no longer visible on the front camera.

    Logic:
    - If SAM3 on the front camera does NOT find `object_name` → the object
      is occluded by the container (went inside) → SUCCESS.
    - If SAM3 DOES find it on the front camera → the object is sitting on
      the floor or somewhere reachable by the front camera, not in the
      container → FAILURE (Gap A: agent lied about the drop).

    This is a coarse but robust check for "drop into a bin" style placement.
    For "drop onto a surface" (e.g. onto a kitchen table), this check would
    incorrectly flag failure because the object stays visible. Use only when
    the target is a container the object falls INTO.
    """
    try:
        seg_raw = await mcp.call_tool_prefixed(
            "perception__segment_objects",
            {"prompt": object_name, "camera": "front"},
        )
        seg = json.loads(seg_raw) if isinstance(seg_raw, str) else seg_raw
        status = seg.get("status", "UNKNOWN")
    except Exception as e:
        logger.warning(f"  [verify-place] seg error: {e}; treating as ambiguous PASS")
        return True, f"verify seg error: {e}"

    if status == "SUCCESS":
        return (
            False,
            f"object '{object_name}' still segmentable on front camera after "
            f"release — likely did NOT land in the container",
        )
    return (
        True,
        f"object '{object_name}' not visible on front camera — likely inside container",
    )


async def execute_place(
    mcp: MCPClient,
    target_container: str,
    object_name: str = None,
    model: str = None,
    max_tool_calls: int = 25,
) -> dict:
    """Run a place agent to drop the held object into / onto a target.

    Args:
        mcp: Connected MCPClient instance.
        target_container: Natural-language name of the drop target
            (e.g. "basket", "box", "kitchen table").
        object_name: Optional name of the held object. If provided and the
            agent reports SUCCESS, a post-step verifies the object is no
            longer visible on the front camera (i.e. actually dropped in).
        model: LiteLLM model string. Defaults to LLM_MODEL env var.
        max_tool_calls: Safety cap on tool calls to prevent runaway.

    Returns:
        {"success": bool, "reason": str, "tool_calls_used": int}
    """
    all_tools = mcp.get_tools()
    tools = _filter_tools(all_tools) + [CREEP_TOOL]

    system_prompt = _SKILL_FILE.read_text()
    user_message = (
        f"Place the currently held object on/into the '{target_container}' "
        f"in front of you."
    )

    messages = [
        {"role": "system", "content": system_prompt},
        {"role": "user", "content": user_message},
    ]
    tool_call_count = 0

    logger.info(f"Place started: target='{target_container}' object='{object_name}'")

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
            logger.info(f"Place finished: {final_text[:200]}")

            s_matches = list(re.finditer(r'\bSUCCESS\b', final_text, re.IGNORECASE))
            f_matches = list(re.finditer(r'\bFAILURE\b', final_text, re.IGNORECASE))
            last_s = s_matches[-1].start() if s_matches else -1
            last_f = f_matches[-1].start() if f_matches else -1
            success = last_s > last_f

            result = {
                "success": success,
                "reason": final_text,
                "tool_calls_used": tool_call_count,
            }

            if success and object_name:
                verified, info = await _verify_object_placed(mcp, object_name)
                logger.info(f"  [verify-place] {info}")
                if not verified:
                    logger.warning(
                        f"  [verify-place] OVERRIDE: agent claimed SUCCESS but "
                        f"'{object_name}' is still visible on front camera"
                    )
                    result["success"] = False
                    result["reason"] = (
                        f"{final_text}\n\n[verify] OVERRIDE — SUCCESS rejected: {info}."
                    )
                else:
                    result["reason"] = f"{final_text}\n\n[verify] {info}."

            return result
        else:
            stop = response.choices[0].finish_reason
            logger.warning(f"Unexpected finish_reason: {stop}")
            return {
                "success": False,
                "reason": f"Unexpected stop: {stop}",
                "tool_calls_used": tool_call_count,
            }

    logger.warning(f"Place hit max tool calls ({max_tool_calls})")
    return {
        "success": False,
        "reason": f"Exceeded maximum tool calls ({max_tool_calls})",
        "tool_calls_used": tool_call_count,
    }
