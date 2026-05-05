import asyncio
import json
import logging
import math
import re
import time

from multi_agent.llm_client import (
    call_llm, wants_tool_use, is_done, get_tool_calls,
    get_text_content, assistant_message, tool_result_message,
)
from multi_agent.mcp_client import MCPClient
from pathlib import Path

logger = logging.getLogger(__name__)


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


async def _approach_target(
    mcp: MCPClient,
    target_object: str,
    standoff_m: float = 0.85,
) -> tuple[bool, str, int]:
    """Drive to ~standoff_m from the segmented target.

    Reads the cached segmentation centroid (set by the prior
    segment_objects call from _verify_target_visible), projects to map
    frame, computes an approach pose, and dispatches nav2.

    Replaces the per-pick / per-place creep that used to live in the
    manipulation agents. Now navigator owns ALL positioning; pick/place
    are pure manipulation primitives that assume the robot is in reach.

    Standoff default 0.85m places the target at ~0.85m forward in
    base_footprint, comfortably inside UR5's 1.10m practical reach.

    Returns: (success, info, tool_calls_used)
    """
    tool_calls = 0

    # Read cached centroid from the most recent segmentation
    try:
        grasp_raw = await mcp.call_tool_prefixed(
            "perception__get_topdown_grasp_pose",
            {"object_name": target_object},
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
        f"dist={target_dist:.2f}m standoff={standoff_m:.2f}m"
    )

    # Already close enough? skip approach
    if target_dist <= standoff_m + 0.10:
        return (
            True,
            f"target already at {target_dist:.2f}m (≤ standoff {standoff_m:.2f}m + 0.10m); no approach needed",
            tool_calls,
        )

    # Get robot pose in map frame. Try once; on failure, fall back to
    # pure base-frame approach (no map projection needed for short-medium
    # hops). This makes _approach_target resilient to nav2-mcp
    # get_robot_pose timeouts (a recurring issue 2026-05-04).
    rx = ry = ryaw = None
    try:
        pose_raw = await mcp.call_tool_prefixed("nav2__get_robot_pose", {})
        tool_calls += 1
        rx, ry, ryaw = _parse_robot_pose(pose_raw)
    except Exception as e:
        logger.warning(f"  [approach] get_robot_pose failed: {e}")

    if rx is None:
        # FALLBACK: pure base-frame DriveOnHeading. We don't need map
        # frame for the approach — just spin to face target then drive
        # (target_dist - standoff). Lacks nav2's obstacle avoidance, so
        # only use if the path is reasonably clear (fine for kids-room
        # short approaches; risky on long hops with furniture).
        logger.warning(
            f"  [approach] FALLBACK — no map pose; using base-frame "
            f"DriveOnHeading. target_dist={target_dist:.2f}m"
        )
        bearing = math.atan2(target_y_base, target_x_base)
        drive_dist = max(target_dist - standoff_m, 0.20)
        # Spin to face target if bearing is significant
        if abs(bearing) > math.radians(8):
            try:
                await asyncio.wait_for(
                    mcp.call_tool_prefixed(
                        "nav2__spin_robot", {"angle": bearing}
                    ),
                    timeout=15.0,
                )
                tool_calls += 1
                await asyncio.sleep(1.0)
            except asyncio.TimeoutError:
                logger.warning("  [approach] FALLBACK spin to face timed out")
        # Drive forward
        logger.info(
            f"  [approach] FALLBACK DriveOnHeading {drive_dist:.2f}m "
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
            tool_calls += 1
        except asyncio.TimeoutError:
            return (
                False,
                f"FALLBACK DriveOnHeading timed out after 35s",
                tool_calls,
            )
        await asyncio.sleep(1.5)
        return (
            True,
            f"FALLBACK approached via base-frame DriveOnHeading "
            f"({drive_dist:.2f}m forward, bearing {math.degrees(bearing):.1f}°)",
            tool_calls,
        )

    # Project target from base_footprint to map frame
    cos_y, sin_y = math.cos(ryaw), math.sin(ryaw)
    target_map_x = rx + cos_y * target_x_base - sin_y * target_y_base
    target_map_y = ry + sin_y * target_x_base + cos_y * target_y_base

    # Approach pose: standoff_m back from target along robot→target vector
    dx = target_map_x - rx
    dy = target_map_y - ry
    dist_to_target = math.hypot(dx, dy)
    if dist_to_target < 0.01:
        return True, "robot already on target xy", tool_calls
    ux, uy = dx / dist_to_target, dy / dist_to_target
    approach_x = target_map_x - standoff_m * ux
    approach_y = target_map_y - standoff_m * uy
    approach_yaw = math.atan2(dy, dx)
    hop_dist = dist_to_target - standoff_m

    logger.info(
        f"  [approach] target_map=({target_map_x:.2f},{target_map_y:.2f}) "
        f"approach=({approach_x:.2f},{approach_y:.2f},yaw={approach_yaw:.2f}) "
        f"hop={hop_dist:.2f}m"
    )

    NAV_MIN_HOP_DIST = 0.40
    NAV_WALL_TIMEOUT = 60.0

    if hop_dist < NAV_MIN_HOP_DIST:
        # Short hop: spin to face + DriveOnHeading (bypasses nav2 tolerance)
        bearing = math.atan2(target_y_base, target_x_base)
        if abs(bearing) > math.radians(8):
            try:
                await asyncio.wait_for(
                    mcp.call_tool_prefixed(
                        "nav2__spin_robot", {"angle": bearing}
                    ),
                    timeout=15.0,
                )
                tool_calls += 1
                await asyncio.sleep(1.0)
            except asyncio.TimeoutError:
                logger.warning("  [approach] spin to face timed out")

        drive_dist = max(hop_dist, 0.20)
        logger.info(
            f"  [approach] short-hop DriveOnHeading {drive_dist:.2f}m "
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
            tool_calls += 1
        except asyncio.TimeoutError:
            return False, "DriveOnHeading short-hop timed out", tool_calls
    else:
        # Long hop: navigate_to_pose handles spin + drive + obstacle avoidance
        try:
            await asyncio.wait_for(
                mcp.call_tool_prefixed(
                    "nav2__navigate_to_pose",
                    {"x": approach_x, "y": approach_y, "yaw": approach_yaw},
                ),
                timeout=NAV_WALL_TIMEOUT,
            )
            tool_calls += 1
        except asyncio.TimeoutError:
            return (
                False,
                f"navigate_to_pose timed out after {NAV_WALL_TIMEOUT:.0f}s",
                tool_calls,
            )

    # Settle: nav2 reports complete before robot fully decelerates AND
    # before camera buffers flush from the new pose.
    await asyncio.sleep(1.5)

    return (
        True,
        f"approached to ~{standoff_m:.2f}m standoff (hop was {hop_dist:.2f}m)",
        tool_calls,
    )

_SKILL_FILE = Path(__file__).parent / "skills" / "navigator.md"

NAVIGATOR_TOOLS = {
    "moveit__plan_and_execute",
    "nav2__navigate_to_pose",
    "nav2__clear_costmaps",
    "nav2__get_robot_pose",
    "perception__look",
}


def _filter_tools(all_tools: list[dict]) -> list[dict]:
    """Filter to only the tools the navigator needs."""
    return [t for t in all_tools if t["function"]["name"] in NAVIGATOR_TOOLS]


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
    target_object: str,
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
                {"prompt": target_object, "camera": "front", "timeout": 20},
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
            logger.info(f"  [spin-search] FOUND '{target_object}' after {i+1} spins")
            return True, seg.get("description", ""), tool_calls

    logger.info(f"  [spin-search] NOT FOUND after {max_spins} spins")
    return False, None, tool_calls


async def _verify_target_visible(
    mcp: MCPClient, target_object: str
) -> tuple[bool, str, int]:
    """Segmentation gate: target_object MUST be segmented on at least one
    camera (arm preferred, front acceptable) before navigator success is
    accepted.

    Architectural contract (Giovanni-aligned, 2026-04-30): the navigator
    only guarantees that the target is visible to the front camera at
    handoff. The pick/place agent owns the fine-approach step (creep)
    and re-segments on the arm camera once close. So a front-cam
    SUCCESS is sufficient — the pick agent will drive in and resegment.

    Arm-cam SUCCESS is treated as "even better" — it means the robot is
    already in pick range and the handoff is golden.

    Flow:
      1. SAM3 on arm camera; on SUCCESS → accept (best handoff state).
      2. SAM3 on front camera; on SUCCESS → accept (good handoff state —
         navigator will approach next).
      3. If both miss, spin-search up to ``_VERIFY_SPIN_STEPS`` times.
         Each spin: re-check arm cam first, then front cam. Accept the
         first SUCCESS.
      4. If still nothing, return False.

    Returns: (visible, info, tool_calls_used)
    """
    tool_calls = 0

    async def _seg(camera: str) -> str | None:
        nonlocal tool_calls
        try:
            seg_raw = await mcp.call_tool_prefixed(
                "perception__segment_objects",
                {"prompt": target_object, "camera": camera, "timeout": 20},
            )
            tool_calls += 1
            seg = json.loads(seg_raw) if isinstance(seg_raw, str) else seg_raw
            status = seg.get("status", "UNKNOWN")
            logger.info(
                f"  [verify] SAM3 {camera} -> {status} for '{target_object}'"
            )
            return status
        except Exception as e:
            logger.error(f"  [verify] segmentation on {camera} failed: {e}")
            tool_calls += 1
            return None

    # 1: arm cam (best handoff)
    if await _seg("arm") == "SUCCESS":
        return True, "verified on arm camera", tool_calls

    # 2: front cam (acceptable — navigator will approach)
    if await _seg("front") == "SUCCESS":
        return True, "verified on front camera (navigator will approach)", tool_calls

    # 3: spin-and-search — robot likely landed off-axis from nav2
    _VERIFY_SPIN_STEPS = 6
    _VERIFY_SPIN_ANGLE = 1.047  # ~60°
    logger.info(
        f"  [verify] both cameras missed; spin-searching up to "
        f"{_VERIFY_SPIN_STEPS} × {_VERIFY_SPIN_ANGLE:.2f}rad for "
        f"'{target_object}' (arm + front check each spin)"
    )
    for i in range(_VERIFY_SPIN_STEPS):
        try:
            await mcp.call_tool_prefixed(
                "nav2__spin_robot", {"angle": _VERIFY_SPIN_ANGLE}
            )
            tool_calls += 1
        except Exception as e:
            logger.error(f"  [verify] spin {i+1} failed: {e}")
            tool_calls += 1
            continue
        await _wait_until_still(mcp)
        await asyncio.sleep(1.25)  # camera buffer flush after spin
        if await _seg("arm") == "SUCCESS":
            return (
                True,
                f"verified on arm camera after {i+1} spin(s)",
                tool_calls,
            )
        if await _seg("front") == "SUCCESS":
            return (
                True,
                f"verified on front camera after {i+1} spin(s) (navigator will approach)",
                tool_calls,
            )

    return False, "target not segmentable after spin-search", tool_calls


async def _try_spin_search(
    mcp: MCPClient, target_object: str | None, result: dict
) -> dict:
    """Post-process navigator result.

    Architectural contract (Giovanni-aligned, 2026-04-30): the navigator
    is responsible only for landing the robot at a known map coordinate
    AND ensuring the target is visible (front camera minimum, arm
    camera ideal) at handoff. The drive-closer step is OWNED by the
    navigator now does the approach (see _approach_target).
    arm cam and approaches under their own control.

    So the post-process is small:
      - LLM SUCCESS → straight to verify gate (no drive-closer).
      - LLM FAILURE + Check 1 PASS (correct area, target not seen) →
        spin-search to bring it into view, then verify gate.
      - LLM FAILURE + Check 1 FAIL (wrong area) → return FAILURE; the
        orchestrator will replan navigation.

    The verify gate (``_final_verify_gate`` → ``_verify_target_visible``)
    accepts SAM3 SUCCESS on either camera; navigator will approach on arm cam
    from there.
    """
    if not target_object:
        return result

    if result["success"]:
        logger.info(
            f"  LLM saw '{target_object}' via look() — going to verify gate"
        )
        return await _final_verify_gate(mcp, target_object, result)

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
    reason_text = result.get("reason", "")
    check1_match = re.search(
        r"Check\s*1[^\n]*?(PASS|FAIL)", reason_text, re.IGNORECASE
    )
    check1_explicit_fail = bool(
        check1_match and check1_match.group(1).upper() == "FAIL"
    )
    if check1_explicit_fail:
        logger.info(
            f"  Check 1 (area) reported FAIL; skipping spin-search "
            f"(robot is not in the right area)"
        )
        result["reason"] = (
            f"{reason_text}\n\n"
            f"[post] Check 1 (area) failed — skipped spin-search; "
            f"orchestrator should retry navigation."
        )
        return result

    logger.info(
        f"  LLM didn't find '{target_object}' — starting deterministic spin-search"
    )
    found, scene_text, spin_calls = await _spin_search(mcp, target_object)
    result["tool_calls_used"] += spin_calls

    if found:
        logger.info(
            f"  Found '{target_object}' via spin — handing off to verify gate"
        )
        result["success"] = True
        result["reason"] = (
            f"Found '{target_object}' after spin-search. "
            f"Scene: {scene_text[:400] if scene_text else 'N/A'}"
        )
    return await _final_verify_gate(mcp, target_object, result)


async def _final_verify_gate(
    mcp: MCPClient,
    target_object: str,
    result: dict,
) -> dict:
    """Hard gate: a navigator result with success=True is only accepted if
    SAM3 can still segment the target on front or arm camera *right now*.

    Pick / place will use the arm camera, so this is the ground-truth
    contract: the target MUST be SAM3-segmentable at handoff. If both
    cameras + a recovery spin-search miss, success is overturned to
    FAILURE — even when the LLM's look() said it saw the target.
    """
    if not result.get("success"):
        return result
    visible, info, calls = await _verify_target_visible(mcp, target_object)
    result["tool_calls_used"] += calls
    if visible:
        # Now drive to within ~0.85m of the target so pick/place can grasp
        # directly without their own creep step. This was previously each
        # manipulation agent's responsibility (pick/place._creep_closer);
        # consolidating it here makes pick/place pure manipulation.
        approach_ok, approach_info, approach_calls = await _approach_target(
            mcp, target_object, standoff_m=0.85
        )
        result["tool_calls_used"] += approach_calls
        result["reason"] = (
            f"{result['reason']}\n\n[verify] {info}."
            f"\n[approach] {approach_info}."
        )
        if not approach_ok:
            logger.warning(
                f"  [approach] FAILED: {approach_info} — handing off anyway "
                f"(target was visible; pick may still succeed if reachable)"
            )
        return result
    logger.warning(
        f"  [verify] OVERRIDE: navigator claimed SUCCESS but '{target_object}' "
        f"is not segmentable — marking FAILURE"
    )
    result["success"] = False
    result["reason"] = (
        f"{result['reason']}\n\n"
        f"[verify] OVERRIDE — SUCCESS rejected: {info}."
    )
    return result



async def execute_navigate(
    mcp: MCPClient,
    destination: str,
    target_object: str | None = None,
    approach_pose: tuple[float, float, float] | None = None,
    model: str = None,
    max_tool_calls: int = 15,
) -> dict:
    """Run a navigator agent to move the robot to a destination.

    Args:
        mcp: Connected MCPClient instance.
        destination: Natural language description of where to go.
        target_object: Optional name of the object to find at the
                       destination. If provided, navigator must see it
                       before reporting success.
        approach_pose: Optional (x, y, yaw) in the map frame. If provided,
                       the navigator drives directly to this pose. If not,
                       the navigator reasons about where to go.
        model: LiteLLM model string. Defaults to LLM_MODEL env var.
        max_tool_calls: Safety cap on tool calls.

    Returns:
        {"success": bool, "reason": str, "tool_calls_used": int}
    """
    all_tools = mcp.get_tools()
    tools = _filter_tools(all_tools)

    system_prompt = _SKILL_FILE.read_text()

    if approach_pose is not None:
        x, y, yaw = approach_pose
        user_message = (
            f"Navigate to: \"{destination}\"\n"
            f"Approach pose hint (map frame): x={x}, y={y}, yaw={yaw}\n\n"
            "Drive to the approach pose and verify you are at the right "
            "LOCATION (surface + room context) using look(camera=\"front\")."
        )
        max_tool_calls = 15  # arm tuck + nav + look + recovery headroom
        logger.info(
            f"Navigator started (with pose hint): ({x}, {y}, {yaw}) "
            f"dest='{destination}'"
        )
    else:
        target_line = (
            f"\nTarget object to find: \"{target_object}\"\n"
            if target_object else ""
        )
        user_message = (
            f"Navigate to: \"{destination}\"\n"
            f"{target_line}\n"
            "No approach pose provided. Use perception to orient yourself, "
            "reason about where the destination is, navigate there, and "
            "verify arrival."
        )
        logger.info(f"Navigator started (open): dest='{destination}' target='{target_object}'")

    messages = [
        {"role": "system", "content": system_prompt},
        {"role": "user", "content": user_message},
    ]
    tool_call_count = 0

    while tool_call_count < max_tool_calls:
        response = call_llm(messages=messages, tools=tools, model=model)

        if wants_tool_use(response):
            messages.append(assistant_message(response))

            tool_results = []
            for tc_id, tc_name, tc_args in get_tool_calls(response):
                tool_call_count += 1
                logger.info(
                    f"  [nav {tool_call_count}] {tc_name}({json.dumps(tc_args)[:200]})"
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
            logger.info(f"Navigator finished:\n{final_text}")

            # Find last standalone SUCCESS/FAILURE word (not "successfully" etc.)
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
            return await _try_spin_search(mcp, target_object, result)
        else:
            stop = response.choices[0].finish_reason
            logger.warning(f"Unexpected finish_reason: {stop}")
            return {
                "success": False,
                "reason": f"Unexpected stop: {stop}",
                "tool_calls_used": tool_call_count,
            }

    logger.warning(f"Navigator hit max tool calls ({max_tool_calls})")
    result = {
        "success": False,
        "reason": f"Exceeded maximum tool calls ({max_tool_calls})",
        "tool_calls_used": tool_call_count,
    }
    return await _try_spin_search(mcp, target_object, result)
