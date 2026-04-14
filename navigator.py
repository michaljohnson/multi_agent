import json
import logging

from multi_agent.llm_client import (
    call_llm, wants_tool_use, is_done, get_tool_calls,
    get_text_content, assistant_message, tool_result_message,
)
from multi_agent.mcp_client import MCPClient
from multi_agent.prompts import NAVIGATOR_GUIDED_PROMPT, NAVIGATOR_SYSTEM_PROMPT

logger = logging.getLogger(__name__)

# Navigator gets nav2 + perception (to see where it is / what's around)
# NOTE: no segment_objects — it's too permissive (matches any table-like thing).
# describe_scene uses Claude Vision which understands spatial relationships.
NAVIGATOR_TOOLS = [
    "nav2__navigate_to_pose",
    "nav2__get_robot_pose",
    "nav2__get_path_from_robot",
    "nav2__clear_costmaps",
    "nav2__cancel_navigation",
    "perception__describe_scene",
    "moveit__plan_to_named_state",   # needs forward_looking before nav
    "moveit__plan_and_execute",      # needs forward_looking before nav
]


def _filter_tools(all_tools: list[dict]) -> list[dict]:
    """Filter to only the tools the navigator needs."""
    return [t for t in all_tools if t["function"]["name"] in NAVIGATOR_TOOLS]


async def execute_navigate(
    mcp: MCPClient,
    destination: str,
    target_object: str | None = None,
    approach_pose: tuple[float, float, float] | None = None,
    model: str = None,
    max_tool_calls: int = 20,
) -> dict:
    """Run a navigator agent to move the robot to a destination.

    Two modes:

    **Guided mode** (`approach_pose` provided): the navigator is a narrow,
    bounded skill — it drives directly to the given (x, y, yaw) pose and
    verifies arrival by looking for `target_object` in the scene. One
    navigation attempt, one optional clear_costmaps retry. No exploration.
    This is the preferred mode — the orchestrator owns the map.

    **Exploration mode** (no `approach_pose`): legacy free-reasoning mode
    where the navigator tries to find the destination by describing its
    surroundings and picking navigation targets. Unbounded search, prone
    to getting lost in large unknown maps.

    Args:
        mcp: Connected MCPClient instance.
        destination: Natural language description of where to go
                     (e.g. "the workbench with tools on it"). Used for
                     reasoning context.
        target_object: Optional name of the object to be picked/placed at this
                       destination (e.g. "clamp"). Used as the primary arrival
                       signal since vision models describe objects reliably
                       but rename surfaces.
        approach_pose: Optional (x, y, yaw) in the map frame. If provided,
                       activates guided mode — the navigator drives directly
                       to this pose and verifies.
        model: LiteLLM model string. Defaults to LLM_MODEL env var.
        max_tool_calls: Safety cap on tool calls.

    Returns:
        {"success": bool, "reason": str, "tool_calls_used": int}
    """
    all_tools = mcp.get_tools()
    tools = _filter_tools(all_tools)

    if approach_pose is not None:
        # Guided mode — narrow, bounded skill.
        x, y, yaw = approach_pose
        system_prompt = NAVIGATOR_GUIDED_PROMPT
        max_tool_calls = 8  # hard cap: arm tuck + nav + describe + (retry) = ~5
        user_message = (
            f"Approach pose (map frame): x={x}, y={y}, yaw={yaw} (radians)\n"
            f'Destination description: "{destination}"\n\n'
            "Tuck the arm, drive to the approach pose, and verify you are "
            "at the right LOCATION (surface + room context) using "
            "describe_scene. Do NOT try to identify any specific object on "
            "the surface — the executor agent handles that. ONE navigation "
            "attempt only."
        )
        logger.info(
            f"Navigator started (guided): approach=({x}, {y}, {yaw}) "
            f"target='{target_object}' dest='{destination}'"
        )
    else:
        # Exploration mode — legacy free-reasoning path.
        system_prompt = NAVIGATOR_SYSTEM_PROMPT
        user_message = f"Navigate the robot to: {destination}\n\n"
        if target_object:
            user_message += (
                f"Target object at this location: '{target_object}'. "
                f"Use this as your primary arrival signal — if you can see the "
                f"object (or a plausible match for it) in the scene description, "
                f"you are at the right place.\n\n"
            )
        user_message += (
            "Use perception to look around and nav2 to move. "
            "Confirm you have arrived at the right location before reporting success."
        )
        logger.info(
            f"Navigator started (exploration): go to '{destination}'"
            + (f" (target: '{target_object}')" if target_object else "")
        )

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
                    # Log describe_scene results so we can see what the LLM
                    # is reasoning from when it decides whether to stop.
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

            success = "SUCCESS" in final_text.upper()
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

    logger.warning(f"Navigator hit max tool calls ({max_tool_calls})")
    return {
        "success": False,
        "reason": f"Exceeded maximum tool calls ({max_tool_calls})",
        "tool_calls_used": tool_call_count,
    }
