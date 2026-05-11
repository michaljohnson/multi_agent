import json
import logging

from multi_agent.clients.llm import (
    call_llm, wants_tool_use, is_done, get_tool_calls,
    get_text_content, assistant_message, tool_result_message,
)
from multi_agent.clients.mcp import MCPClient
from pathlib import Path

logger = logging.getLogger(__name__)

_SKILL_FILE = Path(__file__).parent / "pick.md"

PICK_TOOLS = {
    "perception__segment_objects",
    "perception__get_topdown_grasp_pose",
    "moveit__plan_and_execute",
    "moveit__get_current_pose",
    "moveit__clear_planning_scene",
    "ros__call_service",
    "ros__send_action_goal",
    "ros__subscribe_once",
}

# Virtual termination tool. The agent calls this as its last action; the
# runtime intercepts the call (it is NOT dispatched to MCP) and returns its
# args as the subagent's result. Replaces regex-scraping of free-text
# "SUCCESS:" / "FAILURE:" sentinels: the agent's structured intent lives in
# tool-call args, not in prose.
REPORT_PICK_RESULT_TOOL = {
    "type": "function",
    "function": {
        "name": "report_pick_result",
        "description": (
            "Finish the pick. The args you pass ARE the subagent's return "
            "value to the orchestrator. Call this exactly once as your "
            "final action. Do NOT emit free-text SUCCESS:/FAILURE: lines."
        ),
        "parameters": {
            "type": "object",
            "required": ["success", "error_code", "reason"],
            "properties": {
                "success": {
                    "type": "boolean",
                    "description": (
                        "True only if /gripper/status reported "
                        "'attached:<target>', the object was lifted, and "
                        "the arm is back at look_forward."
                    ),
                },
                "error_code": {
                    "type": "string",
                    "enum": [
                        "NONE",
                        "PICK_SEG_MISSED",
                        "PICK_REACH_EXCEEDED",
                        "PICK_PLAN_FAILED",
                        "PICK_ATTACH_TIMEOUT",
                        "PICK_WRONG_OBJECT",
                        "PICK_HOLDING_ALREADY",
                    ],
                    "description": "Use NONE on success; otherwise pick the most specific failure code.",
                },
                "reason": {
                    "type": "string",
                    "description": "One or two sentences justifying the result.",
                },
            },
        },
    },
}


def _filter_tools(all_tools: list[dict]) -> list[dict]:
    return [t for t in all_tools if t["function"]["name"] in PICK_TOOLS]


def _parse_seg_status(seg_raw) -> str:
    """Extract 'status' field from segment_objects response (handles str|dict)."""
    if not isinstance(seg_raw, str):
        seg_raw = str(seg_raw)
    try:
        return json.loads(seg_raw).get("status", "UNKNOWN")
    except (json.JSONDecodeError, AttributeError):
        return "UNKNOWN"



async def execute_pick(
    mcp: MCPClient,
    object_name: str,
    model: str = None,
    max_tool_calls: int = 30,
) -> dict:
    """Run a pick agent to grasp an object.

    The robot must already be positioned within UR5 reach of the object.
    Navigator owns ALL positioning (including the fine-approach to ~0.85m
    standoff). Pick is a pure manipulation primitive: segment, grasp,
    verify, lift. If the segmented grasp pose is out of reach, pick
    reports FAILURE — it does not drive the base.

    Args:
        mcp: Connected MCPClient instance.
        object_name: Name of the object to pick (e.g. "red ball").
        model: LiteLLM model string. Defaults to LLM_MODEL env var.
        max_tool_calls: Safety cap on tool calls to prevent runaway.

    Returns:
        {"success": bool, "error_code": str, "reason": str, "tool_calls_used": int}
    """
    all_tools = mcp.get_tools()
    tools = _filter_tools(all_tools) + [REPORT_PICK_RESULT_TOOL]

    system_prompt = _SKILL_FILE.read_text()
    user_message = f"Pick up the object '{object_name}'."

    messages = [
        {"role": "system", "content": system_prompt},
        {"role": "user", "content": user_message},
    ]
    tool_call_count = 0

    logger.info(f"Pick started: '{object_name}'")

    while tool_call_count < max_tool_calls:
        response = call_llm(messages=messages, tools=tools, model=model)

        if wants_tool_use(response):
            # If report_pick_result appears in this batch, terminate
            # immediately with its args; any other tool calls in the same
            # batch are discarded — the subagent's intent is to finish.
            for tc_id, tc_name, tc_args in get_tool_calls(response):
                if tc_name == "report_pick_result":
                    tool_call_count += 1
                    logger.info(
                        f"  [{tool_call_count}] report_pick_result"
                        f"({json.dumps(tc_args)[:200]})"
                    )
                    return {
                        "success": bool(tc_args.get("success", False)),
                        "error_code": tc_args.get("error_code", "NONE"),
                        "reason": tc_args.get("reason", ""),
                        "tool_calls_used": tool_call_count,
                    }

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
            logger.warning(
                f"Pick exited without calling report_pick_result. "
                f"Treating as failure. Last text: {final_text[:500]}"
            )
            return {
                "success": False,
                "error_code": "NONE",
                "reason": (
                    "Subagent exited without calling report_pick_result. "
                    f"Last text: {final_text}"
                ),
                "tool_calls_used": tool_call_count,
            }
        else:
            stop = response.choices[0].finish_reason
            logger.warning(f"Unexpected finish_reason: {stop}")
            return {
                "success": False,
                "error_code": "NONE",
                "reason": f"Unexpected stop: {stop}",
                "tool_calls_used": tool_call_count,
            }

    logger.warning(f"Pick hit max tool calls ({max_tool_calls})")
    return {
        "success": False,
        "error_code": "NONE",
        "reason": f"Exceeded maximum tool calls ({max_tool_calls})",
        "tool_calls_used": tool_call_count,
    }
