import asyncio
import json
import logging
import re
import time

from multi_agent.llm_client import (
    call_llm, wants_tool_use, is_done, get_tool_calls,
    get_text_content, assistant_message, tool_result_message,
)
from multi_agent.mcp_client import MCPClient
from pathlib import Path

logger = logging.getLogger(__name__)

_SKILL_FILE = Path(__file__).parent / "skills" / "navigator.md"

NAVIGATOR_TOOLS = {
    "nav2__navigate_to_pose",
    "nav2__get_robot_pose",
    "nav2__clear_costmaps",
    "nav2__cancel_navigation",
    "perception__describe_scene",
    "perception__segment_objects",
    "perception__get_grasp_from_pointcloud",
    "moveit__plan_and_execute",
}


def _filter_tools(all_tools: list[dict]) -> list[dict]:
    """Filter to only the tools the navigator needs."""
    return [t for t in all_tools if t["function"]["name"] in NAVIGATOR_TOOLS]


_STOP_WORDS = {"the", "a", "an", "on", "in", "at", "of", "from", "near", "by", "to", "is", "it"}


def _object_in_scene(target: str, scene_text: str) -> bool:
    """Check if target object appears in describe_scene result.

    Parses the JSON to extract the 'objects' list and 'description',
    then uses difflib to fuzzy-match target words against each listed
    object. This avoids false positives from common English words in
    free text (e.g. 'can' as a verb).
    """
    from difflib import SequenceMatcher

    target_words = [w for w in target.lower().split() if w not in _STOP_WORDS]
    if not target_words:
        return False

    # Try to parse as JSON to get the objects list
    objects_list = []
    description = ""
    try:
        data = json.loads(scene_text) if isinstance(scene_text, str) else scene_text
        objects_list = data.get("objects", [])
        description = data.get("description", "")
    except (json.JSONDecodeError, TypeError):
        description = str(scene_text).lower()

    # Check each object in the structured list
    for obj in objects_list:
        obj_lower = str(obj).lower()
        obj_words = obj_lower.replace("_", " ").replace("/", " ").split()
        # Count how many target words have a good fuzzy match in this object
        matched = 0
        for tw in target_words:
            for ow in obj_words:
                ratio = SequenceMatcher(None, tw, ow).ratio()
                if ratio >= 0.7:  # 70% similarity
                    matched += 1
                    break
        if matched >= max(2, len(target_words) // 2):
            logger.info(f"  object match: '{obj}' matched {matched}/{len(target_words)} target words")
            return True

    # Fallback: check description for color + object type pattern
    desc_lower = description.lower()
    color_words = [w for w in target_words if w in {"red", "blue", "green", "black", "white", "yellow", "orange", "brown", "pink", "grey", "gray"}]
    if color_words and any(cw in desc_lower for cw in color_words):
        # Found the color — check if there's also an object-like word nearby
        object_words = [w for w in target_words if w not in _STOP_WORDS and w not in color_words and len(w) > 2]
        for ow in object_words:
            if ow in desc_lower:
                logger.info(f"  object match: color '{color_words}' + '{ow}' found in description")
                return True

    logger.info(f"  object match: no match for '{target}' in {len(objects_list)} objects")
    return False


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
    the pointcloud is already cached and ready for _segment_and_approach_front.

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


async def _approach_forward(
    mcp: MCPClient,
    target_object: str,
    max_steps: int = 3,
    step_distance: float = 0.3,
) -> int:
    """Drive forward in small steps toward the object the robot is facing.

    After spin-search found the object, the robot is already facing it.
    Drive forward step_distance at a time, checking describe_scene after
    each step to confirm the object is still visible (getting closer).

    Uses /amcl_pose for current position (get_robot_pose is stale).

    Returns tool_calls_used.
    """
    import math
    tool_calls = 0

    for step in range(max_steps):
        # Get current pose
        try:
            pose_raw = await mcp.call_tool_prefixed(
                "ros__subscribe_once",
                {"topic": "/amcl_pose", "msg_type": "geometry_msgs/msg/PoseWithCovarianceStamped", "timeout": 3},
            )
            tool_calls += 1
            pose_data = json.loads(pose_raw) if isinstance(pose_raw, str) else pose_raw
            if "msg" in pose_data:
                pose_data = pose_data["msg"]
            pos = pose_data["pose"]["pose"]["position"]
            ori = pose_data["pose"]["pose"]["orientation"]
            siny = 2.0 * (ori["w"] * ori["z"] + ori["x"] * ori["y"])
            cosy = 1.0 - 2.0 * (ori["y"] ** 2 + ori["z"] ** 2)
            yaw = math.atan2(siny, cosy)

            target_x = pos["x"] + step_distance * math.cos(yaw)
            target_y = pos["y"] + step_distance * math.sin(yaw)

            logger.info(
                f"  [approach {step+1}/{max_steps}] driving {step_distance}m forward: "
                f"({pos['x']:.2f}, {pos['y']:.2f}) → ({target_x:.2f}, {target_y:.2f})"
            )
            await mcp.call_tool_prefixed(
                "nav2__navigate_to_pose",
                {"x": round(target_x, 2), "y": round(target_y, 2), "yaw": round(yaw, 2)},
            )
            tool_calls += 1
        except Exception as e:
            logger.error(f"  [approach] step {step+1} failed: {e}")
            tool_calls += 1
            break

    return tool_calls


async def _segment_once(
    mcp: MCPClient, target_object: str, camera: str = "front"
) -> tuple[float | None, float | None, int]:
    """Run SAM3 on the named camera + grasp lookup; return (base_x, base_y, calls)."""
    calls = 0
    try:
        seg_raw = await mcp.call_tool_prefixed(
            "perception__segment_objects",
            {"prompt": target_object, "camera": camera, "timeout": 30},
        )
        calls += 1
        seg = json.loads(seg_raw) if isinstance(seg_raw, str) else seg_raw
        if seg.get("status") != "SUCCESS":
            logger.info(f"  [seg:{camera}] returned {seg.get('status')}")
            return None, None, calls
    except Exception as e:
        logger.error(f"  [seg:{camera}] segment_objects failed: {e}")
        return None, None, calls

    try:
        grasp_raw = await mcp.call_tool_prefixed(
            "perception__get_grasp_from_pointcloud",
            {"object_name": target_object},
        )
        calls += 1
        grasp = json.loads(grasp_raw) if isinstance(grasp_raw, str) else grasp_raw
        c = grasp.get("centroid_base_frame", {})
        return float(c.get("x", 0.0)), float(c.get("y", 0.0)), calls
    except Exception as e:
        logger.error(f"  [seg:{camera}] get_grasp failed: {e}")
        return None, None, calls


async def _segment_and_approach_front(
    mcp: MCPClient,
    target_object: str,
    standoff_distance: float = 1.00,
    max_iterations: int = 2,
    done_threshold: float = 0.20,
    bearing_tolerance: float = 0.08,  # ~4.6° — then lateral < ~5cm at 0.55m
) -> tuple[bool, int]:
    """Center on the target, then drive to ``standoff_distance`` from it.

    Each iteration:
      1. Segment target on front camera.
      2. If |bearing| > ``bearing_tolerance``, spin by bearing to center it
         (nav2_spin_robot is more precise than packing yaw into nav2 goal).
      3. Re-segment after the spin.
      4. Drive to the approach point in map frame via nav2_navigate_to_pose.
      5. Done when distance is within ``done_threshold`` of ``standoff_distance``.

    Keeps rotation and translation separate so neither fights the other's
    tolerance. Uses front camera + TF for base-frame math (AMCL-independent);
    AMCL is only used once per hop to convert the translation target to map.
    """
    import math

    tool_calls = 0
    last_distance = None

    for step in range(max_iterations):
        # (1) Segment to locate the target in base_footprint (front camera only)
        base_x, base_y, c = await _segment_once(mcp, target_object, camera="front")
        tool_calls += c
        if base_x is None:
            # Front camera didn't see it. Arm-camera verify only counts if
            # it actually sees the target. If both cameras miss, report
            # failure and let the verify-gate spin-search handle it —
            # never silently declare success here (that was the lenient
            # escape that produced false positives).
            if step > 0:
                arm_x, arm_y, arm_c = await _segment_once(
                    mcp, target_object, camera="arm"
                )
                tool_calls += arm_c
                if arm_x is not None:
                    arm_dist = math.hypot(arm_x, arm_y)
                    logger.info(
                        f"  [approach-seg {step+1}] front miss in blind spot, "
                        f"but arm-camera verified at ({arm_x:.2f}, {arm_y:.2f}) "
                        f"d={arm_dist:.2f}m — done"
                    )
                    return True, tool_calls
                # Only accept as done if the LAST known distance was
                # already within ~standoff (i.e., we actually arrived).
                if last_distance is not None and last_distance <= standoff_distance + done_threshold:
                    logger.info(
                        f"  [approach-seg {step+1}] both cameras miss but last "
                        f"distance {last_distance:.2f}m was within standoff; done"
                    )
                    return True, tool_calls
                logger.info(
                    f"  [approach-seg {step+1}] front+arm miss and last distance "
                    f"{last_distance}m not within standoff — FAILURE"
                )
                return False, tool_calls
            logger.info(
                f"  [approach-seg {step+1}/{max_iterations}] target not visible"
            )
            return False, tool_calls

        distance = math.hypot(base_x, base_y)
        last_distance = distance
        bearing = math.atan2(base_y, base_x)
        logger.info(
            f"  [approach-seg {step+1}/{max_iterations}] '{target_object}' at "
            f"base_footprint=({base_x:.2f}, {base_y:.2f}) "
            f"d={distance:.2f}m bearing={math.degrees(bearing):.1f}°"
        )

        # (2) Center by spinning if bearing is off
        if abs(bearing) > bearing_tolerance:
            logger.info(f"  [approach-seg {step+1}] spin {math.degrees(bearing):.1f}° to center")
            try:
                await mcp.call_tool_prefixed("nav2__spin_robot", {"angle": bearing})
                tool_calls += 1
            except Exception as e:
                logger.error(f"  [approach-seg {step+1}] spin_robot failed: {e}")
                return False, tool_calls

            # Wait until robot is physically still before re-segmenting
            await _wait_until_still(mcp)

            # (3) Re-segment after spin (front camera only)
            base_x, base_y, c = await _segment_once(mcp, target_object, camera="front")
            tool_calls += c
            if base_x is None:
                logger.info(
                    f"  [approach-seg {step+1}] target lost after centering spin"
                )
                # Still consider the current position usable if we got close
                return (last_distance is not None and last_distance <= standoff_distance + 0.5), tool_calls
            distance = math.hypot(base_x, base_y)
            last_distance = distance
            logger.info(
                f"  [approach-seg {step+1}] centered: base=({base_x:.2f}, {base_y:.2f}) d={distance:.2f}m"
            )

        # Are we already at standoff?
        if abs(distance - standoff_distance) <= done_threshold:
            logger.info(
                f"  [approach-seg] within {done_threshold}m of {standoff_distance}m standoff"
            )
            return True, tool_calls

        if distance < 1e-3:
            logger.warning("  [approach-seg] object on top of robot; aborting")
            return False, tool_calls

        # (4) Drive forward along the (now-centered) robot→object ray.
        # Cap the per-iteration hop so we re-segment and correct heading
        # before committing to the final approach. Long single hops
        # compound AMCL drift + path replanning + yaw error.
        _MAX_HOP_DISTANCE = 1.5  # metres
        remaining = distance - standoff_distance  # >0 because checked above
        hop = min(remaining, _MAX_HOP_DISTANCE)
        t = hop / distance
        approach_base_x = base_x * t
        approach_base_y = base_y * t
        if hop < remaining:
            logger.info(
                f"  [approach-seg {step+1}] long approach ({remaining:.2f}m); "
                f"capping this hop to {hop:.2f}m and will re-segment"
            )

        # Read current AMCL pose for base→map conversion (one-shot; drift
        # across a short hop is small)
        try:
            pose_raw = await mcp.call_tool_prefixed(
                "ros__subscribe_once",
                {
                    "topic": "/amcl_pose",
                    "msg_type": "geometry_msgs/msg/PoseWithCovarianceStamped",
                    "timeout": 3,
                },
            )
            tool_calls += 1
            pose_data = json.loads(pose_raw) if isinstance(pose_raw, str) else pose_raw
            if "msg" in pose_data:
                pose_data = pose_data["msg"]
            p = pose_data["pose"]["pose"]["position"]
            o = pose_data["pose"]["pose"]["orientation"]
            siny = 2.0 * (o["w"] * o["z"] + o["x"] * o["y"])
            cosy = 1.0 - 2.0 * (o["y"] ** 2 + o["z"] ** 2)
            robot_yaw = math.atan2(siny, cosy)
            robot_x, robot_y = p["x"], p["y"]
        except Exception as e:
            logger.error(f"  [approach-seg {step+1}] amcl_pose failed: {e}")
            return False, tool_calls

        cos_y, sin_y = math.cos(robot_yaw), math.sin(robot_yaw)
        map_x = robot_x + approach_base_x * cos_y - approach_base_y * sin_y
        map_y = robot_y + approach_base_x * sin_y + approach_base_y * cos_y
        # Goal yaw: face the ball from the goal position. nav2's yaw
        # tolerance is loose after long drives; computing yaw from goal→ball
        # guarantees the robot points at the target at arrival.
        ball_map_x = robot_x + base_x * cos_y - base_y * sin_y
        ball_map_y = robot_y + base_x * sin_y + base_y * cos_y
        map_yaw = math.atan2(ball_map_y - map_y, ball_map_x - map_x)

        logger.info(
            f"  [approach-seg {step+1}] nav2 → map ({map_x:.2f}, {map_y:.2f}, "
            f"yaw={map_yaw:.2f})"
        )
        try:
            await mcp.call_tool_prefixed(
                "nav2__navigate_to_pose",
                {
                    "x": round(map_x, 3),
                    "y": round(map_y, 3),
                    "yaw": round(map_yaw, 3),
                },
            )
            tool_calls += 1
        except Exception as e:
            logger.error(f"  [approach-seg {step+1}] nav2 failed: {e}")
            return False, tool_calls

        # Wait until robot stops decelerating before next iteration's segment
        await _wait_until_still(mcp)

    logger.info(
        f"  [approach-seg] iterations exhausted; last distance={last_distance:.2f}m"
    )
    return (last_distance is not None and last_distance < standoff_distance + 0.5), tool_calls


async def _verify_target_visible(
    mcp: MCPClient, target_object: str
) -> tuple[bool, str, int]:
    """Hard segmentation gate: target_object MUST be segmented on at least
    one camera before navigator success is accepted.

    Flow:
      1. SAM3 on front camera; on SUCCESS → accept.
      2. SAM3 on arm camera; on SUCCESS → accept.
      3. If both miss, spin-and-search up to ``_VERIFY_SPIN_STEPS`` times
         (front camera each spin). The robot often lands off-axis after
         nav2, so the ball sits just outside the FOV — a short spin
         sequence recovers it without giving up.
      4. If still nothing, return False so _final_verify_gate flips
         success=False.

    SAM3 returns no mask if the object isn't there, so this is ground
    truth, not opinion.

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

    # 1+2: stationary check on both cameras
    for camera in ("front", "arm"):
        if await _seg(camera) == "SUCCESS":
            return True, f"verified on {camera} camera", tool_calls

    # 3: spin-and-search — robot likely landed off-axis from nav2
    _VERIFY_SPIN_STEPS = 6
    _VERIFY_SPIN_ANGLE = 1.047  # ~60°
    logger.info(
        f"  [verify] both cameras missed; spin-searching up to "
        f"{_VERIFY_SPIN_STEPS} × {_VERIFY_SPIN_ANGLE:.2f}rad for "
        f"'{target_object}'"
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
        if await _seg("front") == "SUCCESS":
            return (
                True,
                f"verified on front camera after {i+1} spin(s)",
                tool_calls,
            )

    return False, "target not segmentable after spin-search", tool_calls


async def _try_spin_search(
    mcp: MCPClient, target_object: str | None, result: dict
) -> dict:
    """Post-process navigator result with deterministic helpers.

    Flow:
    - If no target_object, return as-is.
    - If LLM already found the target (success=True), run the front-camera
      segmentation approach helper to place the robot at grasping standoff.
    - Otherwise, run spin-search; if it finds the target, also run the
      segmentation approach helper.
    """
    if not target_object:
        return result

    if result["success"]:
        logger.info(
            f"  LLM reported SUCCESS for '{target_object}' — refining approach "
            f"via front segmentation"
        )
        approach_ok, approach_calls = await _segment_and_approach_front(mcp, target_object)
        result["tool_calls_used"] += approach_calls
        if approach_ok:
            result["reason"] = (
                f"{result['reason']}\n\n"
                f"[post] Front-camera approach refined — robot at standoff."
            )
            return await _final_verify_gate(mcp, target_object, result)

        # Target not in front view from the initial nav pose — try spin-search
        # to rotate until SAM3 sees it, then approach again.
        logger.info(
            f"  Target not in initial front view — spin-searching for '{target_object}'"
        )
        found, _info, spin_calls = await _spin_search(mcp, target_object)
        result["tool_calls_used"] += spin_calls
        if found:
            approach_ok, approach_calls = await _segment_and_approach_front(
                mcp, target_object
            )
            result["tool_calls_used"] += approach_calls
            result["reason"] = (
                f"{result['reason']}\n\n"
                f"[post] Target not in initial view; found after spin-search. "
                f"Approach: {'refined' if approach_ok else 'failed'}."
            )
        else:
            result["success"] = False
            result["reason"] = (
                f"{result['reason']}\n\n"
                f"[post] Area confirmed, but target not found after spin-search."
            )
        return await _final_verify_gate(mcp, target_object, result)

    logger.info(f"  LLM didn't find '{target_object}' — starting deterministic spin-search")
    found, scene_text, spin_calls = await _spin_search(mcp, target_object)
    result["tool_calls_used"] += spin_calls

    if found:
        logger.info(f"  Found '{target_object}' — refining approach via front segmentation")
        approach_ok, approach_calls = await _segment_and_approach_front(mcp, target_object)
        result["tool_calls_used"] += approach_calls
        result["success"] = True
        result["reason"] = (
            f"Found '{target_object}' after spin-search. "
            f"Segmentation approach: {'completed' if approach_ok else 'failed/skipped'}. "
            f"Scene: {scene_text[:400] if scene_text else 'N/A'}"
        )
    return await _final_verify_gate(mcp, target_object, result)


async def _final_verify_gate(
    mcp: MCPClient, target_object: str, result: dict
) -> dict:
    """Hard gate: a navigator result with success=True is only accepted if
    SAM3 can still segment the target_object on the front or arm camera
    *right now*. Catches LLM hallucinations and lenient approach escapes."""
    if not result.get("success"):
        return result
    visible, info, calls = await _verify_target_visible(mcp, target_object)
    result["tool_calls_used"] += calls
    if visible:
        result["reason"] = f"{result['reason']}\n\n[verify] {info}."
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
    max_tool_calls: int = 8,
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
            "LOCATION (surface + room context) using describe_scene."
        )
        max_tool_calls = 8  # bounded: arm tuck + nav + describe + retry
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
                    result = await mcp.call_tool_prefixed(tc_name, tc_args)
                    if tc_name == "perception__describe_scene":
                        logger.info(
                            f"  [nav {tool_call_count}] describe_scene -> {result[:800]}"
                        )
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
            logger.info(f"Navigator finished: {final_text[:200]}")

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
