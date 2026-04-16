import json
import logging
import re

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


async def _spin_search(
    mcp: MCPClient,
    target_object: str,
    max_spins: int = 5,
    spin_angle: float = 1.05,
) -> tuple[bool, str | None, int]:
    """Deterministic spin-and-search loop (no LLM involved).

    Spins the robot ~60° at a time and calls describe_scene after each spin.
    Stops early if the target object is found.

    Returns:
        (found, last_scene_text, tool_calls_used)
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

        # Describe
        try:
            scene_text = await mcp.call_tool_prefixed("perception__describe_scene", {})
            tool_calls += 1
            logger.info(f"  [spin-search {i+1}/{max_spins}] describe_scene -> {scene_text[:600]}")
        except Exception as e:
            logger.error(f"  [spin-search] describe_scene failed: {e}")
            tool_calls += 1
            continue

        # Check
        if _object_in_scene(target_object, scene_text):
            logger.info(f"  [spin-search] FOUND '{target_object}' after {i+1} spins")
            return True, scene_text, tool_calls

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


async def _try_spin_search(
    mcp: MCPClient, target_object: str | None, result: dict
) -> dict:
    """If LLM failed but target_object is set, try deterministic spin-search.
    If found, drive forward to get closer."""
    if not target_object or result["success"]:
        return result

    logger.info(f"  LLM didn't find '{target_object}' — starting deterministic spin-search")
    found, scene_text, spin_calls = await _spin_search(mcp, target_object)
    result["tool_calls_used"] += spin_calls

    if found:
        # Drive forward toward the object (robot is facing it after spin)
        logger.info(f"  Found '{target_object}' — approaching with forward steps")
        approach_calls = await _approach_forward(mcp, target_object)
        result["tool_calls_used"] += approach_calls
        result["success"] = True
        result["reason"] = (
            f"Found '{target_object}' after spin-search and approached it. "
            f"Scene: {scene_text[:500] if scene_text else 'N/A'}"
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
