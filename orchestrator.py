import json
import logging

from multi_agent.llm_client import (
    call_llm, wants_tool_use, is_done, get_tool_calls,
    get_text_content, assistant_message, tool_result_message,
)
from multi_agent.mcp_client import MCPClient
from multi_agent.executor import execute_pick_and_place
from multi_agent.navigator import execute_navigate
from multi_agent.prompts import ORCHESTRATOR_SYSTEM_PROMPT

logger = logging.getLogger(__name__)


# Custom tools the orchestrator can call (not MCP tools — Python-implemented)
ORCHESTRATOR_TOOLS = [
    {
        "type": "function",
        "function": {
            "name": "navigate",
            "description": (
                "Navigate the robot to a destination. Spawns a navigator skill "
                "that drives directly to a given approach_pose and verifies by "
                "target_object. The navigator is narrow and bounded — it does "
                "NOT explore. You (the orchestrator) own the map and must "
                "provide approach_pose in the map frame. Pass target_object so "
                "the navigator can verify arrival by seeing the object."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "destination": {
                        "type": "string",
                        "description": (
                            "Natural language description of the destination "
                            "(used for logging / context only, NOT for reasoning)."
                        ),
                    },
                    "target_object": {
                        "type": "string",
                        "description": (
                            "Name of the object to pick or place at this location "
                            "(e.g. 'clamp'). Used as the arrival signal — the "
                            "navigator reports SUCCESS if this object is visible."
                        ),
                    },
                    "approach_pose": {
                        "type": "array",
                        "items": {"type": "number"},
                        "minItems": 3,
                        "maxItems": 3,
                        "description": (
                            "(x, y, yaw) pose in the map frame. The navigator "
                            "drives directly to this pose — no exploration. "
                            "You must look up the correct pose from your map "
                            "knowledge for the requested destination."
                        ),
                    },
                },
                "required": ["destination", "target_object", "approach_pose"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "pick",
            "description": (
                "Pick up an object that is near the robot. The robot must already "
                "be positioned close enough to reach the object. Spawns an executor "
                "agent that handles perception, grasp planning, and manipulation."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "object_name": {
                        "type": "string",
                        "description": "Name of the object to pick up (e.g. 'clamp', 'tablet')",
                    },
                },
                "required": ["object_name"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "place",
            "description": (
                "Place the currently held object on a surface in front of the robot. "
                "The robot must already be positioned near the target surface. "
                "Spawns an executor agent that lowers the object, releases, and retracts."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "surface_height": {
                        "type": "number",
                        "description": "Height of the target surface in meters (e.g. 0.80 for a table)",
                    },
                },
                "required": ["surface_height"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "get_robot_pose",
            "description": "Get the current robot position and orientation in the map frame.",
            "parameters": {
                "type": "object",
                "properties": {},
            },
        },
    },
]


async def _handle_tool_call(
    mcp: MCPClient,
    tool_name: str,
    tool_input: dict,
    executor_model: str,
    navigator_model: str,
) -> str:
    """Handle a tool call from the orchestrator."""
    if tool_name == "navigate":
        # approach_pose comes over JSON as a list; cast to tuple for the navigator.
        raw_pose = tool_input.get("approach_pose")
        approach_pose = tuple(raw_pose) if raw_pose is not None else None
        result = await execute_navigate(
            mcp=mcp,
            destination=tool_input["destination"],
            target_object=tool_input.get("target_object"),
            approach_pose=approach_pose,
            model=navigator_model,
        )
        return json.dumps(result)

    elif tool_name == "pick":
        result = await execute_pick_and_place(
            mcp=mcp,
            object_name=tool_input["object_name"],
            mode="pick",
            model=executor_model,
        )
        return json.dumps(result)

    elif tool_name == "place":
        result = await execute_pick_and_place(
            mcp=mcp,
            object_name="held object",
            mode="place",
            surface_height=tool_input["surface_height"],
            model=executor_model,
        )
        return json.dumps(result)

    elif tool_name == "get_robot_pose":
        result = await mcp.call_tool("nav2-mcp-server", "get_robot_pose", {})
        return result

    else:
        return f"Unknown tool: {tool_name}"


async def run_orchestrator(
    mcp: MCPClient,
    task_list: list[dict],
    location_map: dict[str, tuple[float, float, float]],
    orchestrator_model: str = None,
    executor_model: str = None,
    navigator_model: str = None,
    max_turns: int = 50,
) -> dict:
    """Run the orchestrator agent to clean a room.

    Args:
        mcp: Connected MCPClient instance.
        task_list: List of dicts with 'object', 'pickup_location',
                   'place_location', and 'surface_height' keys.
        location_map: Dict of known location names -> (x, y, yaw) approach
                      poses in the map frame. Injected into the user
                      message so the orchestrator LLM knows which exact
                      approach_pose to pass to the navigate tool.
        orchestrator_model: LiteLLM model string. Defaults to LLM_MODEL env var.
        executor_model: LiteLLM model string. Defaults to LLM_MODEL env var.
        navigator_model: LiteLLM model string. Defaults to LLM_MODEL env var.
        max_turns: Safety cap on orchestrator turns.

    Returns:
        {"summary": str, "turns_used": int}
    """
    # Build user message from task list
    task_descriptions = []
    for i, task in enumerate(task_list, 1):
        task_descriptions.append(
            f"{i}. Pick up the '{task['object']}' from {task['pickup_location']} "
            f"and place it on {task['place_location']} "
            f"(surface height: {task['surface_height']}m)"
        )

    # Known locations with exact approach poses — the orchestrator must
    # reuse these exact values when calling the navigate tool. Without
    # this, the LLM has no source for the required approach_pose field
    # and will hallucinate coordinates.
    location_lines = [
        f"- {name}: approach_pose=({x}, {y}, {yaw})"
        for name, (x, y, yaw) in location_map.items()
    ]
    known_locations_block = (
        "Known locations (use these EXACT approach_pose values when calling "
        "the navigate tool — do NOT invent coordinates):\n"
        + "\n".join(location_lines)
    )

    user_message = (
        "Clean the room by picking up the following objects and placing them "
        "at their target locations:\n\n"
        + "\n".join(task_descriptions)
        + "\n\n"
        + known_locations_block
    )

    messages = [
        {"role": "system", "content": ORCHESTRATOR_SYSTEM_PROMPT},
        {"role": "user", "content": user_message},
    ]
    turn_count = 0

    logger.info(f"Orchestrator started with {len(task_list)} objects")

    while turn_count < max_turns:
        turn_count += 1

        response = call_llm(
            messages=messages,
            tools=ORCHESTRATOR_TOOLS,
            model=orchestrator_model,
        )

        if wants_tool_use(response):
            messages.append(assistant_message(response))

            tool_results = []
            for tc_id, tc_name, tc_args in get_tool_calls(response):
                logger.info(
                    f"Orchestrator -> {tc_name}({json.dumps(tc_args)[:200]})"
                )

                result = await _handle_tool_call(
                    mcp, tc_name, tc_args,
                    executor_model, navigator_model,
                )
                tool_results.append(tool_result_message(tc_id, result))

            messages.extend(tool_results)

        elif is_done(response):
            final_text = get_text_content(response)
            logger.info(f"Orchestrator finished:\n{final_text}")
            return {
                "summary": final_text,
                "turns_used": turn_count,
            }
        else:
            stop = response.choices[0].finish_reason
            logger.warning(f"Unexpected finish_reason: {stop}")
            return {
                "summary": f"Unexpected stop: {stop}",
                "turns_used": turn_count,
            }

    logger.warning(f"Orchestrator hit max turns ({max_turns})")
    return {
        "summary": f"Exceeded maximum turns ({max_turns})",
        "turns_used": turn_count,
    }
