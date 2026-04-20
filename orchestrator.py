import json
import logging

from multi_agent.llm_client import (
    call_llm, wants_tool_use, is_done, get_tool_calls,
    get_text_content, assistant_message, tool_result_message,
)
from multi_agent.mcp_client import MCPClient
from multi_agent.pick import execute_pick
from multi_agent.place import execute_place
from multi_agent.navigator import execute_navigate
from pathlib import Path

logger = logging.getLogger(__name__)

_SKILL_FILE = Path(__file__).parent / "skills" / "orchestrator.md"


# Custom tools the orchestrator can call (Python-implemented, not MCP)
ORCHESTRATOR_TOOLS = [
    {
        "type": "function",
        "function": {
            "name": "navigate",
            "description": (
                "Navigate the robot to a destination. Spawns a navigator agent "
                "that uses its own environment knowledge to drive there and "
                "verifies arrival using perception."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "destination": {
                        "type": "string",
                        "description": (
                            "Natural language description of where to go "
                            "(e.g. 'the wooden coffee table in the living room')."
                        ),
                    },
                },
                "required": ["destination"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "pick",
            "description": (
                "Pick up an object near the robot. The robot must already be "
                "positioned close enough to reach it (call navigate first). "
                "Spawns a pick-and-place agent that handles perception, grasp "
                "planning, and manipulation."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "object_name": {
                        "type": "string",
                        "description": "Name of the object to pick up.",
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
                "Place the currently held object onto / into a target "
                "container or surface. The robot must already be near the "
                "target (call navigate first). Spawns a place agent that "
                "segments the target on the front camera, computes its "
                "centroid, lowers the object above it, releases, and retracts."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "target_container": {
                        "type": "string",
                        "description": (
                            "Natural-language name of the drop target "
                            "(e.g. 'basket', 'cardboard box', 'kitchen table')."
                        ),
                    },
                },
                "required": ["target_container"],
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
        result = await execute_navigate(
            mcp=mcp,
            destination=tool_input["destination"],
            model=navigator_model,
        )
        return json.dumps(result)

    elif tool_name == "pick":
        result = await execute_pick(
            mcp=mcp,
            object_name=tool_input["object_name"],
            model=executor_model,
        )
        return json.dumps(result)

    elif tool_name == "place":
        result = await execute_place(
            mcp=mcp,
            target_container=tool_input["target_container"],
            model=executor_model,
        )
        return json.dumps(result)

    else:
        return f"Unknown tool: {tool_name}"


async def run_orchestrator(
    mcp: MCPClient,
    task: str,
    orchestrator_model: str = None,
    executor_model: str = None,
    navigator_model: str = None,
    max_turns: int = 50,
) -> dict:
    """Run the orchestrator agent with an open-ended task.

    Args:
        mcp: Connected MCPClient instance.
        task: Natural language task description (e.g. "pick up the clamp
              from the coffee table and put it on the kitchen table").
        orchestrator_model: LiteLLM model string.
        executor_model: LiteLLM model string for pick-and-place agents.
        navigator_model: LiteLLM model string for navigator agents.
        max_turns: Safety cap on orchestrator turns.

    Returns:
        {"summary": str, "turns_used": int}
    """
    system_prompt = _SKILL_FILE.read_text()

    messages = [
        {"role": "system", "content": system_prompt},
        {"role": "user", "content": task},
    ]
    turn_count = 0

    logger.info(f"Orchestrator started: {task[:200]}")

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
