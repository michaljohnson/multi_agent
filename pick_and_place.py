import json
import logging

from multi_agent.llm_client import (
    call_llm, wants_tool_use, is_done, get_tool_calls,
    get_text_content, assistant_message, tool_result_message,
)
from multi_agent.mcp_client import MCPClient
from pathlib import Path

logger = logging.getLogger(__name__)

_SKILL_FILE = Path(__file__).parent / "skills" / "pick_and_place.md"

PICK_AND_PLACE_TOOLS = {
    "perception__segment_objects",
    "perception__get_grasp_from_pointcloud",
    "moveit__plan_and_execute",
    "moveit__get_current_pose",
    "moveit__clear_planning_scene",
    "ros__call_service",
    "ros__send_action_goal",
    "ros__publish_once",
    "ros__subscribe_once",
    "nav2__navigate_to_pose",
}


def _filter_tools(all_tools: list[dict]) -> list[dict]:
    """Filter to only the tools the pick-and-place agent needs."""
    return [t for t in all_tools if t["function"]["name"] in PICK_AND_PLACE_TOOLS]


async def execute_pick_and_place(
    mcp: MCPClient,
    object_name: str,
    mode: str = "pick",
    surface_height: float = 0.80,
    model: str = None,
    max_tool_calls: int = 30,
) -> dict:
    """Run a pick-and-place agent to pick or place an object.

    Args:
        mcp: Connected MCPClient instance.
        object_name: Name of the object to pick (e.g. "scissors").
        mode: "pick" to pick up an object, "place" to place the held object.
        surface_height: Height of the target surface (used in place mode).
        model: LiteLLM model string. Defaults to LLM_MODEL env var.
        max_tool_calls: Safety cap on tool calls to prevent runaway.

    Returns:
        {"success": bool, "reason": str, "tool_calls_used": int}
    """
    all_tools = mcp.get_tools()
    tools = _filter_tools(all_tools)

    system_prompt = _SKILL_FILE.read_text()

    if mode == "pick":
        user_message = f"Mode: PICK\nPick up the object '{object_name}'."
    else:
        user_message = (
            f"Mode: PLACE\n"
            f"Place the currently held object on the surface in front of you. "
            f"The surface height is approximately {surface_height}m."
        )

    messages = [
        {"role": "system", "content": system_prompt},
        {"role": "user", "content": user_message},
    ]
    tool_call_count = 0

    logger.info(f"Pick-and-place started ({mode}): '{object_name}'")

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
            logger.info(f"Pick-and-place finished: {final_text[:200]}")

            success = final_text.upper().startswith("SUCCESS")
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

    logger.warning(f"Pick-and-place hit max tool calls ({max_tool_calls})")
    return {
        "success": False,
        "reason": f"Exceeded maximum tool calls ({max_tool_calls})",
        "tool_calls_used": tool_call_count,
    }
