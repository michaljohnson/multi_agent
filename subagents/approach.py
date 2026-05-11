import asyncio
import json
import logging
import math
import time

from multi_agent.clients.llm import (
    call_llm, wants_tool_use, is_done, get_tool_calls,
    get_text_content, assistant_message, tool_result_message,
)
from multi_agent.clients.mcp import MCPClient
from pathlib import Path

logger = logging.getLogger(__name__)

_SKILL_FILE = Path(__file__).parent / "approach.md"

APPROACH_TOOLS = {
    "moveit__plan_and_execute",
    "nav2__navigate_to_pose",
    "nav2__clear_costmaps",
    "nav2__get_robot_pose",
    "perception__look",
}

# Virtual termination tool. The agent calls this as its last action; the
# runtime intercepts the call (it is NOT dispatched to MCP) and returns its
# args as the subagent's result. Replaces regex-scraping of free-text
# "SUCCESS:" / "FAILURE:" sentinels and the separate Check 1 / Check 2
# PASS/FAIL extractor: both pieces of state are now structured tool-call
# args (success bool + checks[] array), not prose to be parsed.
REPORT_APPROACH_RESULT_TOOL = {
    "type": "function",
    "function": {
        "name": "report_approach_result",
        "description": (
            "Finish the navigation. The args you pass ARE the subagent's "
            "return value to the orchestrator. Call this exactly once as "
            "your final action. Do NOT emit free-text SUCCESS:/FAILURE: "
            "lines anymore. The structured 'checks' array replaces the "
            "old 'Check 1 (area): PASS/FAIL — ...' reasoning block."
        ),
        "parameters": {
            "type": "object",
            "required": ["success", "error_code", "reason", "checks"],
            "properties": {
                "success": {
                    "type": "boolean",
                    "description": (
                        "True only if Check 1 (area) PASS AND Check 2 (target) PASS."
                    ),
                },
                "error_code": {
                    "type": "string",
                    "enum": [
                        "NONE",
                        "NAV_AREA_WRONG",
                        "NAV_TARGET_NOT_VISIBLE",
                        "NAV_DRIVE_FAILED",
                        "NAV_PLAN_FAILED",
                        "NAV_VERIFY_OVERRIDE",
                    ],
                    "description": "Use NONE on success; otherwise pick the most specific failure code.",
                },
                "reason": {
                    "type": "string",
                    "description": "One or two sentences justifying the result.",
                },
                "checks": {
                    "type": "array",
                    "description": (
                        "Per-check breakdown. Always include both entries "
                        "(name='area' and name='target') with result in "
                        "{PASS, FAIL}. An object_name is always supplied."
                    ),
                    "items": {
                        "type": "object",
                        "required": ["name", "result"],
                        "properties": {
                            "name": {"type": "string", "enum": ["area", "target"]},
                            "result": {"type": "string", "enum": ["PASS", "FAIL"]},
                            "note": {"type": "string"},
                        },
                    },
                },
            },
        },
    },
}


def _filter_tools(all_tools: list[dict]) -> list[dict]:
    """Filter to only the tools the approach agent needs."""
    return [t for t in all_tools if t["function"]["name"] in APPROACH_TOOLS]


def _check_result(checks: list[dict], name: str) -> str:
    """Return the 'result' field for a named check ('PASS' / 'FAIL' / 'SKIP'),
    or 'UNKNOWN' if the check is missing.
    """
    for c in checks or []:
        if c.get("name") == name:
            return str(c.get("result", "UNKNOWN")).upper()
    return "UNKNOWN"


def _parse_robot_pose(raw) -> tuple[float | None, float | None, float | None]:
    """Extract (x, y, yaw) from nav2__get_robot_pose response. Returns
    (None, None, None) on parse failure. Same logic as pick.py."""
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


# Next-action-aware standoff lookup. The right standoff depends on what
# the next agent will do once handoff completes:
#   - pick: UR5 reaches forward at low z (~0.40m) — 0.85m centroid
#     distance leaves comfortable headroom for grasp pose math.
#   - surface_place: wrist must be HIGH (surface_z + 0.31m for can on
#     coffee table = 0.66m). UR5 top-down reach at z=0.66m caps at
#     x≈0.55m, so approach agent needs to deliver closer (~0.45m).
#   - container_place: drop INTO the bin from above; wrist sits 35cm
#     above rim. Same UR5 high-z constraints apply but the rim is
#     usually at moderate height, so 0.65m gives margin.
#   - floor_place: similar to pick — soft set-down at low z.
_STANDOFF_BY_NEXT_ACTION = {
    "pick": 0.85,
    "surface_place": 0.45,
    "container_place": 0.65,
    "floor_place": 0.85,
}


async def _approach_target(
    mcp: MCPClient,
    object_name: str,
    standoff_m: float = 0.85,
) -> tuple[bool, str, int]:
    """Drive to ~standoff_m from the segmented target.

    Reads the cached segmentation centroid (set by the prior
    segment_objects call in ``_final_verify_gate`` or in
    ``_spin_search``), then delegates to the shared
    ``nav2__approach_target`` MCP primitive.

    All three architectures (LLM + tools, multi-agent, skill-based) call
    the same primitive to ensure they implement approach identically.
    The primitive owns geometry + motion (spin to face + drive_on_heading);
    this wrapper only bridges perception (centroid lookup) to nav2.

    Returns: (success, info, tool_calls_used)
    """
    tool_calls = 0

    # Read cached centroid from perception (TF snapshot taken at segment time)
    try:
        grasp_raw = await mcp.call_tool_prefixed(
            "perception__get_topdown_grasp_pose",
            {"object_name": object_name},
        )
        tool_calls += 1
        grasp = json.loads(grasp_raw) if isinstance(grasp_raw, str) else grasp_raw
        target_x_base = float(grasp["centroid_base_frame"]["x"])
        target_y_base = float(grasp["centroid_base_frame"]["y"])
    except (json.JSONDecodeError, KeyError, TypeError, ValueError) as e:
        return False, f"failed to read cached centroid: {e}", tool_calls

    target_dist = math.hypot(target_x_base, target_y_base)
    logger.info(
        f"  [approach] target_base=({target_x_base:.2f},{target_y_base:.2f}) "
        f"dist={target_dist:.2f}m standoff={standoff_m:.2f}m -> nav2__approach_target"
    )

    # Delegate to the shared MCP primitive
    try:
        result_raw = await asyncio.wait_for(
            mcp.call_tool_prefixed(
                "nav2__approach_target",
                {
                    "target_x_base": target_x_base,
                    "target_y_base": target_y_base,
                    "standoff_m": standoff_m,
                },
            ),
            timeout=60.0,
        )
        tool_calls += 1
    except asyncio.TimeoutError:
        return False, "nav2__approach_target wall-timeout after 60s", tool_calls
    except Exception as e:
        return False, f"nav2__approach_target error: {e}", tool_calls

    text = result_raw if isinstance(result_raw, str) else str(result_raw)
    if "error" in text.lower():
        return False, text, tool_calls

    # Settle: nav2 reports complete before robot fully decelerates AND
    # before camera buffers flush from the new pose.
    await asyncio.sleep(1.5)

    return True, text, tool_calls


async def _wait_until_still(
    mcp: MCPClient,
    timeout: float = 3.0,
    vel_threshold: float = 0.02,
    poll_s: float = 0.15,
    post_settle: float = 0.25,
) -> None:
    """Await robot stillness by polling /odom twist.

    nav2 reports spin/navigate complete as soon as it stops *commanding*,
    but the velocity_smoother keeps decelerating for ~500ms. If we segment
    during that decel, the image is mid-motion and pose is off the goal.

    Polls /odom twist until |linear.x| and |angular.z| are under
    `vel_threshold`, then waits `post_settle` for the camera ring buffer
    to flush (10Hz → latest cached frame may be up to 100ms old).
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            raw = await mcp.call_tool_prefixed(
                "ros__subscribe_once",
                {
                    "topic": "/odom",
                    "msg_type": "nav_msgs/msg/Odometry",
                    "timeout": 1,
                },
            )
            data = json.loads(raw) if isinstance(raw, str) else raw
            msg = data.get("msg", data)
            tw = msg.get("twist", {}).get("twist", {})
            vx = abs(tw.get("linear", {}).get("x", 0.0))
            wz = abs(tw.get("angular", {}).get("z", 0.0))
            if vx < vel_threshold and wz < vel_threshold:
                await asyncio.sleep(post_settle)
                return
        except Exception as e:
            logger.debug(f"  [wait-still] odom poll error: {e}")
        await asyncio.sleep(poll_s)
    logger.warning(f"  [wait-still] timeout after {timeout}s; proceeding anyway")
    await asyncio.sleep(post_settle)


async def _spin_search(
    mcp: MCPClient,
    object_name: str,
    max_spins: int = 8,
    spin_angle: float = 1.047,
) -> tuple[bool, str | None, int]:
    """Deterministic spin-and-search loop (no LLM involved).

    Spins the robot ~60° at a time and calls SAM3 segmentation on the front
    camera after each spin. SAM3 returns SUCCESS when it actually finds the
    object (pixel-precise), so no fuzzy text matching is needed. On success,
    control returns to the verify gate; pick/place owns the drive-closer
    step from there.

    Returns:
        (found, last_detection_info, tool_calls_used)
    """
    tool_calls = 0
    for i in range(max_spins):
        # Spin
        try:
            await mcp.call_tool_prefixed("nav2__spin_robot", {"angle": spin_angle})
            tool_calls += 1
            logger.info(f"  [spin-search {i+1}/{max_spins}] spun {spin_angle:.2f} rad")
        except Exception as e:
            logger.error(f"  [spin-search] spin failed: {e}")
            tool_calls += 1
            continue

        # Wait until robot is physically still before segmenting
        await _wait_until_still(mcp)

        # Segment on the front camera (SAM3)
        try:
            seg_raw = await mcp.call_tool_prefixed(
                "perception__segment_objects",
                {"prompt": object_name, "camera": "front", "timeout": 20},
            )
            tool_calls += 1
            seg = json.loads(seg_raw) if isinstance(seg_raw, str) else seg_raw
            status = seg.get("status", "UNKNOWN")
            logger.info(
                f"  [spin-search {i+1}/{max_spins}] SAM3 front -> {status}"
            )
        except Exception as e:
            logger.error(f"  [spin-search] SAM3 segmentation failed: {e}")
            tool_calls += 1
            continue

        if status == "SUCCESS":
            logger.info(f"  [spin-search] FOUND '{object_name}' after {i+1} spins")
            return True, seg.get("description", ""), tool_calls

    logger.info(f"  [spin-search] NOT FOUND after {max_spins} spins")
    return False, None, tool_calls


async def _try_spin_search(
    mcp: MCPClient,
    object_name: str,
    result: dict,
    standoff_m: float = 0.85,
) -> dict:
    """Post-process approach agent result.

    Architectural contract: the approach agent is responsible for landing the
    robot at a known map coordinate AND ensuring the target is visible
    (front camera minimum, arm camera ideal) at handoff. The drive-closer
    step is also owned by the approach agent (see ``_approach_target``);
    pick/place are pure manipulation primitives once handoff completes.

    So the post-process is small:
      - LLM SUCCESS → straight to verify gate (no drive-closer here).
      - LLM FAILURE + Check 1 PASS (correct area, target not seen) →
        spin-search to bring it into view, then verify gate.
      - LLM FAILURE + Check 1 FAIL (wrong area) → return FAILURE; the
        orchestrator will replan navigation.

    The final gate (``_final_verify_gate``) trusts the agent's own
    look()+LLM-vision verification from step 3 (no runtime SAM3 re-verify).
    It uses SAM3 only for COORDINATES (one segment call to populate the
    centroid cache), then drives in via ``_approach_target``.
    """
    if result["success"]:
        logger.info(
            f"  LLM saw '{object_name}' via look() — driving to standoff"
        )
        return await _final_verify_gate(mcp, object_name, result, standoff_m=standoff_m)

    # LLM did not see the target. Three failure modes:
    #   - Check 1 explicitly FAIL: wrong area / never reached destination.
    #     Spinning here is wasted motion (target is presumably elsewhere).
    #     Report FAILURE so the orchestrator can replan.
    #   - Check 1 PASS, Check 2 FAIL: in the right area but target not
    #     visible from current angle. Spin-search makes sense.
    #   - Check 1 NEITHER (max tool calls hit, parse failure, LLM error):
    #     we don't actually know if we're in the right area. Spin-search
    #     is cheap insurance — try it. If we're in the wrong room SAM3
    #     just won't anchor, and the gate FAILS cleanly anyway.
    area_check = _check_result(result.get("checks", []), "area")
    if area_check == "FAIL":
        logger.info(
            f"  Check 1 (area) reported FAIL; skipping spin-search "
            f"(robot is not in the right area)"
        )
        result["reason"] = (
            f"{result.get('reason', '')}\n\n"
            f"[post] Check 1 (area) failed — skipped spin-search; "
            f"orchestrator should retry navigation."
        )
        if result.get("error_code", "NONE") == "NONE":
            result["error_code"] = "NAV_AREA_WRONG"
        return result

    logger.info(
        f"  LLM didn't find '{object_name}' — starting deterministic spin-search"
    )
    found, scene_text, spin_calls = await _spin_search(mcp, object_name)
    result["tool_calls_used"] += spin_calls

    if found:
        logger.info(
            f"  Found '{object_name}' via spin — handing off to verify gate"
        )
        result["success"] = True
        result["error_code"] = "NONE"
        result["reason"] = (
            f"Found '{object_name}' after spin-search. "
            f"Scene: {scene_text[:400] if scene_text else 'N/A'}"
        )
        # Spin-search recovery puts the target back in view; reflect that
        # in the structured target check so the result is internally
        # consistent (success=True must not coexist with target=FAIL).
        # The note records that this PASS came from the deterministic
        # post-step, not from the agent's own observation.
        for c in result.get("checks", []):
            if c.get("name") == "target":
                c["result"] = "PASS"
                c["note"] = (
                    f"recovered via runtime spin-search after the agent "
                    f"reported FAIL: " + str(c.get("note", ""))
                )
                break
    return await _final_verify_gate(mcp, object_name, result, standoff_m=standoff_m)


async def _final_verify_gate(
    mcp: MCPClient,
    object_name: str,
    result: dict,
    standoff_m: float = 0.85,
) -> dict:
    """Drive to standoff on agent success.

    No runtime SAM3 re-verification of the agent's success claim — the
    agent's own `look(camera="both")` + LLM-vision check (step 3) IS the
    verification. SAM3 is unreliable for "is the target visible here
    right now?" yes/no questions (see
    `feedback_sam3_vs_look_primitive_selection.md`), so we trust the
    LLM-vision check and only drive in. If the agent was wrong, the
    downstream pick/place will fail at its own reach/visibility gate
    and the orchestrator will re-call approach.

    Args:
        standoff_m: Target distance (meters) from segmented target after
            the approach hop. Default 0.85m for pick. Surface place
            should pass 0.45m, container place 0.65m. See
            _STANDOFF_BY_NEXT_ACTION for the next_action → standoff mapping.
    """
    if not result.get("success"):
        return result

    # SAM3 segment to populate the cached centroid that _approach_target
    # reads. SAM3 is used here for COORDINATES only (centroid xy for the
    # approach driver), NOT as a yes/no visibility gate — the agent's
    # own look(camera="both") + LLM-vision in step 3 is the verification.
    # Arm cam preferred (best handoff state); front cam fallback.
    segged_on = None
    for camera in ("arm", "front"):
        try:
            seg_raw = await mcp.call_tool_prefixed(
                "perception__segment_objects",
                {"prompt": object_name, "camera": camera, "timeout": 20},
            )
            result["tool_calls_used"] += 1
            seg = json.loads(seg_raw) if isinstance(seg_raw, str) else seg_raw
            if seg.get("status") == "SUCCESS":
                segged_on = camera
                logger.info(
                    f"  [coords] SAM3 {camera} -> SUCCESS (centroid cached)"
                )
                break
            else:
                logger.info(
                    f"  [coords] SAM3 {camera} -> {seg.get('status', 'UNKNOWN')}"
                )
        except Exception as e:
            logger.error(f"  [coords] segment on {camera} failed: {e}")
            result["tool_calls_used"] += 1

    if segged_on is None:
        logger.warning(
            f"  [coords] SAM3 missed '{object_name}' on both cams — "
            f"skipping approach drive. Agent's look() said target was visible, "
            f"so handing off at the current pose; pick/place may still succeed."
        )
        result["reason"] = (
            f"{result['reason']}\n\n"
            f"[coords] SAM3 missed on both cams; no approach drive."
        )
        return result

    # Drive to within `standoff_m` of the target so pick/place can
    # grasp/release directly without their own creep step.
    approach_ok, approach_info, approach_calls = await _approach_target(
        mcp, object_name, standoff_m=standoff_m
    )
    result["tool_calls_used"] += approach_calls
    result["reason"] = (
        f"{result['reason']}\n\n"
        f"[coords] segmented on {segged_on} cam."
        f"\n[approach] {approach_info}."
    )
    if not approach_ok:
        logger.warning(
            f"  [approach] FAILED: {approach_info} — handing off anyway "
            f"(agent reported target visible; pick may still succeed if reachable)"
        )
    return result



async def execute_approach(
    mcp: MCPClient,
    target_area: str,
    object_name: str,
    next_action: str,
    approach_pose: tuple[float, float, float] | None = None,
    model: str = None,
    max_tool_calls: int = 15,
) -> dict:
    """Run an approach agent to move the robot to a destination area.

    Args:
        mcp: Connected MCPClient instance.
        target_area: Natural language description of the destination area
                       (e.g. "kids room", "living room near the coffee table").
        object_name: Name of the object the approach agent must see at the
                       target_area before reporting success. Required —
                       an approach agent with no specific target has nothing
                       to verify against; use a relocate-only tool if
                       pure repositioning without target verification
                       is what you want.
        approach_pose: Optional (x, y, yaw) in the map frame. If provided,
                       the approach agent drives directly to this pose. If not,
                       the approach agent reasons about where to go.
        next_action: What the next subagent call will be. Determines
              `_approach_target` standoff via _STANDOFF_BY_NEXT_ACTION lookup.
              Required — picking the wrong next_action silently delivers the
              robot at the wrong standoff and the downstream pick/place
              fails at its reach gate. Valid values: 'pick' (0.85m),
              'surface_place' (0.45m), 'container_place' (0.65m),
              'floor_place' (0.85m).
        model: LiteLLM model string. Defaults to LLM_MODEL env var.
        max_tool_calls: Safety cap on tool calls.

    Returns:
        {"success": bool, "error_code": str, "reason": str,
         "checks": list[dict], "tool_calls_used": int}
    """
    if not object_name:
        raise ValueError(
            "execute_approach requires a non-empty object_name. The "
            "approach agent's job is to put the target in view; without one "
            "it cannot verify arrival."
        )

    standoff_m = _STANDOFF_BY_NEXT_ACTION.get(next_action, 0.85)
    if next_action not in _STANDOFF_BY_NEXT_ACTION:
        logger.warning(
            f"Unknown approach next_action '{next_action}'; falling back to 0.85m standoff"
        )

    all_tools = mcp.get_tools()
    tools = _filter_tools(all_tools) + [REPORT_APPROACH_RESULT_TOOL]

    system_prompt = _SKILL_FILE.read_text()

    if approach_pose is not None:
        x, y, yaw = approach_pose
        user_message = (
            f"Navigate to: \"{target_area}\"\n"
            f"Target object to find: \"{object_name}\"\n"
            f"Approach pose hint (map frame): x={x}, y={y}, yaw={yaw}\n\n"
            "Drive to the approach pose and verify you are at the right "
            "LOCATION (surface + room context) using look(camera=\"front\")."
        )
        max_tool_calls = 15  # arm tuck + nav + look + recovery headroom
        logger.info(
            f"Approach started (with pose hint): ({x}, {y}, {yaw}) "
            f"area='{target_area}' target='{object_name}'"
        )
    else:
        user_message = (
            f"Navigate to: \"{target_area}\"\n"
            f"Target object to find: \"{object_name}\"\n\n"
            "No approach pose provided. Use perception to orient yourself, "
            "reason about where the target area is, drive there, and "
            "verify arrival."
        )
        logger.info(f"Approach started (open): area='{target_area}' target='{object_name}'")

    messages = [
        {"role": "system", "content": system_prompt},
        {"role": "user", "content": user_message},
    ]
    tool_call_count = 0

    while tool_call_count < max_tool_calls:
        response = call_llm(messages=messages, tools=tools, model=model)

        if wants_tool_use(response):
            # If report_approach_result appears in this batch, terminate
            # immediately with its args; any other tool calls in the same
            # batch are discarded — the subagent's intent is to finish.
            for tc_id, tc_name, tc_args in get_tool_calls(response):
                if tc_name == "report_approach_result":
                    tool_call_count += 1
                    logger.info(
                        f"  [{tool_call_count}] report_approach_result"
                        f"({json.dumps(tc_args)[:300]})"
                    )
                    result = {
                        "success": bool(tc_args.get("success", False)),
                        "error_code": tc_args.get("error_code", "NONE"),
                        "reason": tc_args.get("reason", ""),
                        "checks": tc_args.get("checks", []),
                        "tool_calls_used": tool_call_count,
                    }
                    return await _try_spin_search(
                        mcp, object_name, result, standoff_m=standoff_m
                    )

            messages.append(assistant_message(response))

            tool_results = []
            for tc_id, tc_name, tc_args in get_tool_calls(response):
                tool_call_count += 1
                logger.info(
                    f"  [{tool_call_count}] {tc_name}({json.dumps(tc_args)[:200]})"
                )

                try:
                    if tc_name == "perception__look":
                        # MCP `look` returns image content blocks; preserve
                        # them as-is so the LLM sees the actual pixels.
                        result = await mcp.call_tool_prefixed_raw(tc_name, tc_args)
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
            logger.warning(
                f"Approach exited without calling report_approach_result. "
                f"Treating as failure. Last text: {final_text[:500]}"
            )
            result = {
                "success": False,
                "error_code": "NONE",
                "reason": (
                    "Subagent exited without calling report_approach_result. "
                    f"Last text: {final_text}"
                ),
                "checks": [],
                "tool_calls_used": tool_call_count,
            }
            return await _try_spin_search(
                mcp, object_name, result, standoff_m=standoff_m
            )
        else:
            stop = response.choices[0].finish_reason
            logger.warning(f"Unexpected finish_reason: {stop}")
            return {
                "success": False,
                "error_code": "NONE",
                "reason": f"Unexpected stop: {stop}",
                "checks": [],
                "tool_calls_used": tool_call_count,
            }

    logger.warning(f"Approach hit max tool calls ({max_tool_calls})")
    result = {
        "success": False,
        "error_code": "NONE",
        "reason": f"Exceeded maximum tool calls ({max_tool_calls})",
        "checks": [],
        "tool_calls_used": tool_call_count,
    }
    return await _try_spin_search(mcp, object_name, result)
