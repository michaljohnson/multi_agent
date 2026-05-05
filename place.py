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


def _filter_tools(all_tools: list[dict]) -> list[dict]:
    return [t for t in all_tools if t["function"]["name"] in PLACE_TOOLS]



def _parse_seg_status(seg_raw) -> str:
    if not isinstance(seg_raw, str):
        seg_raw = str(seg_raw)
    try:
        return json.loads(seg_raw).get("status", "UNKNOWN")
    except (json.JSONDecodeError, AttributeError):
        return "UNKNOWN"


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

    The robot must already be positioned within UR5 reach of the target
    container/surface. Navigator owns ALL positioning (including the
    fine-approach to ~0.85m standoff). Place is a pure manipulation
    primitive: segment, compute drop pose, lower, release, retract.
    If the segmented drop pose is out of reach, place reports FAILURE
    — it does not drive the base.

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
    tools = _filter_tools(all_tools)

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
            logger.info(f"Place finished:\n{final_text}")

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
