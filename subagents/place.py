import json
import logging

from multi_agent.clients.llm import (
    call_llm, wants_tool_use, is_done, get_tool_calls,
    get_text_content, assistant_message, tool_result_message,
)
from multi_agent.clients.mcp import MCPClient
from pathlib import Path

logger = logging.getLogger(__name__)

_SKILL_FILE = Path(__file__).parent / "place.md"

PLACE_TOOLS = {
    "perception__segment_objects",
    "perception__get_topdown_placing_pose",
    "perception__look",
    "moveit__plan_and_execute",
    "moveit__get_current_pose",
    "moveit__clear_planning_scene",
    "ros__call_service",
    "ros__send_action_goal",
    "ros__publish_once",
}

# Virtual termination tool. The agent calls this as its last action; the
# runtime intercepts the call (it is NOT dispatched to MCP) and returns its
# args as the subagent's result. Replaces regex-scraping of free-text
# "SUCCESS:" / "FAILURE:" sentinels: the agent's structured intent lives in
# tool-call args, not in prose.
REPORT_PLACE_RESULT_TOOL = {
    "type": "function",
    "function": {
        "name": "report_place_result",
        "description": (
            "Finish the place. The args you pass ARE the subagent's return "
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
                        "True only if the held object was released at the "
                        "intended drop pose and the arm retracted to "
                        "look_forward. A post-step visibility verifier may "
                        "still override this to false."
                    ),
                },
                "error_code": {
                    "type": "string",
                    "enum": [
                        "NONE",
                        "PLACE_OUT_OF_SCOPE",
                        "PLACE_SEG_MISSED",
                        "PLACE_REACH_EXCEEDED",
                        "PLACE_PLAN_FAILED",
                        "PLACE_HOLDING_NOTHING",
                        "PLACE_DROP_VERIFY_FAILED",
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
    return [t for t in all_tools if t["function"]["name"] in PLACE_TOOLS]



def _parse_seg_status(seg_raw) -> str:
    if not isinstance(seg_raw, str):
        seg_raw = str(seg_raw)
    try:
        return json.loads(seg_raw).get("status", "UNKNOWN")
    except (json.JSONDecodeError, AttributeError):
        return "UNKNOWN"


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
        object_name: Optional name of the held object. When provided it
            is included in the agent's user_message so step 12 (the
            agent-side look-down vision verify gate) knows what to look
            for in the arm camera image. When omitted, the agent has no
            label for step 12 and should report success=false with
            error_code=PLACE_DROP_VERIFY_FAILED unless it can otherwise
            confirm the drop landed in the target.
        model: LiteLLM model string. Defaults to LLM_MODEL env var.
        max_tool_calls: Safety cap on tool calls to prevent runaway.

    Returns:
        {"success": bool, "error_code": str, "reason": str, "tool_calls_used": int}
    """
    all_tools = mcp.get_tools()
    tools = _filter_tools(all_tools) + [REPORT_PLACE_RESULT_TOOL]

    system_prompt = _SKILL_FILE.read_text()
    if object_name:
        user_message = (
            f"Place the currently held '{object_name}' on/into the "
            f"'{target_container}' in front of you. Use '{object_name}' as "
            f"the SAM3 prompt for the step 12 look-down verification."
        )
    else:
        user_message = (
            f"Place the currently held object on/into the '{target_container}' "
            f"in front of you. NOTE: no object_name was supplied, so the "
            f"step 12 look-down verification gate cannot run — report "
            f"success=false, error_code=PLACE_DROP_VERIFY_FAILED if you "
            f"cannot otherwise confirm the drop landed in the target."
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
            # If report_place_result appears in this batch, terminate
            # immediately with its args; any other tool calls in the same
            # batch are discarded — the subagent's intent is to finish.
            for tc_id, tc_name, tc_args in get_tool_calls(response):
                if tc_name == "report_place_result":
                    tool_call_count += 1
                    logger.info(
                        f"  [{tool_call_count}] report_place_result"
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
                f"Place exited without calling report_place_result. "
                f"Treating as failure. Last text: {final_text[:500]}"
            )
            return {
                "success": False,
                "error_code": "NONE",
                "reason": (
                    "Subagent exited without calling report_place_result. "
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

    logger.warning(f"Place hit max tool calls ({max_tool_calls})")
    return {
        "success": False,
        "error_code": "NONE",
        "reason": f"Exceeded maximum tool calls ({max_tool_calls})",
        "tool_calls_used": tool_call_count,
    }
