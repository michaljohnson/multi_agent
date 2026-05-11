import json
import logging

from multi_agent.clients.llm import (
    call_llm, wants_tool_use, is_done, get_tool_calls,
    get_text_content, assistant_message, tool_result_message,
)
from multi_agent.clients.mcp import MCPClient
from multi_agent.subagents.pick import execute_pick
from multi_agent.subagents.place import execute_place
from multi_agent.subagents.approach import execute_approach
from pathlib import Path

logger = logging.getLogger(__name__)

_SKILL_FILE = Path(__file__).parent / "orchestrator.md"


# Custom tools the orchestrator can call (Python-implemented, not MCP)
ORCHESTRATOR_TOOLS = [
    {
        "type": "function",
        "function": {
            "name": "approach",
            "description": (
                "Find a named object in a target area and approach it. "
                "Spawns an approach agent that drives to the area, visually "
                "verifies arrival, finds the object (spin-searching if "
                "needed), and closes to a mode-dependent standoff so the "
                "next step (pick or place) starts within arm-reach. The "
                "`mode` argument selects the standoff: 0.85m for pick / "
                "floor_place, 0.65m for container_place, 0.45m for "
                "surface_place (UR5 needs to be close for top-down release "
                "at table height)."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "target_area": {
                        "type": "string",
                        "description": (
                            "Natural language description of the destination "
                            "area (e.g. 'the kids room', 'the living room near "
                            "the coffee table')."
                        ),
                    },
                    "object_name": {
                        "type": "string",
                        "description": (
                            "Required. The visible landmark the approach "
                            "agent must put in view (SAM3-verified) and close "
                            "to the standoff. Before a floor pickup: pass the "
                            "object itself (e.g. 'red ball'). Before a place: "
                            "pass the container or surface name (e.g. "
                            "'trash bin', 'wooden coffee table'). "
                            "(Surface pickups do NOT call approach at all — "
                            "the user pre-positions the robot next to the "
                            "surface.)"
                        ),
                    },
                    "mode": {
                        "type": "string",
                        "enum": [
                            "pick",
                            "surface_place",
                            "container_place",
                            "floor_place",
                        ],
                        "description": (
                            "What the next agent will do after this approach "
                            "completes. Determines approach standoff: "
                            "'pick' / 'floor_place' = 0.85m, "
                            "'container_place' = 0.65m (drop-into bin), "
                            "'surface_place' = 0.45m (UR5 needs close standoff "
                            "for top-down release at table height — surface "
                            "place fails reach if standoff > 0.55m). "
                            "Required: pick the value matching the NEXT "
                            "subagent call you intend to make."
                        ),
                    },
                },
                "required": ["target_area", "object_name", "mode"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "pick",
            "description": (
                "Pick up an object near the robot. The robot must already be "
                "positioned close enough to reach it (call approach first). "
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
                "target (call approach first). Spawns a place agent that "
                "segments the target on the front camera, computes its "
                "centroid, releases the object above it, retracts, and "
                "verifies the drop via an arm-cam look-down vision check "
                "(step 12 in place.md)."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "target_location": {
                        "type": "string",
                        "description": (
                            "Natural-language name of the drop target "
                            "(e.g. 'basket', 'cardboard box', 'kitchen table')."
                        ),
                    },
                    "object_name": {
                        "type": "string",
                        "description": (
                            "Name of the held object. Used by step 12 "
                            "(visual look-down verify) as the label when "
                            "deciding whether the released object landed "
                            "in/on the target. Always pass the same name "
                            "the prior pick() call used."
                        ),
                    },
                },
                "required": ["target_location", "object_name"],
            },
        },
    },
]

# MCP tools the orchestrator is allowed to call directly (alongside its
# Python-implemented sub-agent tools above).
ORCHESTRATOR_MCP_TOOLS = {
    "perception__look",
}


async def _handle_tool_call(
    mcp: MCPClient,
    tool_name: str,
    tool_input: dict,
    executor_model: str,
    approach_model: str,
):
    """Handle a tool call from the orchestrator.

    For sub-agent tools (approach / pick / place) returns a JSON string.
    For raw MCP tools (look, etc.) returns the raw content blocks.
    """
    if tool_name == "approach":
        result = await execute_approach(
            mcp=mcp,
            target_area=tool_input["target_area"],
            object_name=tool_input["object_name"],
            mode=tool_input["mode"],
            model=approach_model,
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
            target_location=tool_input["target_location"],
            object_name=tool_input["object_name"],
            model=executor_model,
        )
        return json.dumps(result)

    # MCP-routed tools (look, future direct-MCP tools) — go through the
    # raw call path so image content blocks survive into the LLM message.
    return await mcp.call_tool_prefixed_raw(tool_name, tool_input)


async def run_orchestrator(
    mcp: MCPClient,
    task: str,
    orchestrator_model: str = None,
    executor_model: str = None,
    approach_model: str = None,
    max_turns: int = 50,
) -> dict:
    """Run the orchestrator agent with an open-ended task.

    Args:
        mcp: Connected MCPClient instance.
        task: Natural language task description (e.g. "pick up the clamp
              from the coffee table and put it on the kitchen table").
        orchestrator_model: LiteLLM model string.
        executor_model: LiteLLM model string for pick-and-place agents.
        approach_model: LiteLLM model string for the approach agent.
        max_turns: Safety cap on orchestrator turns.

    Returns:
        {"summary": str, "turns_used": int}
    """
    system_prompt = _SKILL_FILE.read_text()
    all_mcp_tools = mcp.get_tools()
    direct_mcp = [
        t for t in all_mcp_tools
        if t["function"]["name"] in ORCHESTRATOR_MCP_TOOLS
    ]
    all_tools = ORCHESTRATOR_TOOLS + direct_mcp

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
            tools=all_tools,
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
                    executor_model, approach_model,
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
