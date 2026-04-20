import json
import logging
import math
import re

from multi_agent.llm_client import (
    call_llm, wants_tool_use, is_done, get_tool_calls,
    get_text_content, assistant_message, tool_result_message,
)
from multi_agent.mcp_client import MCPClient
from pathlib import Path

logger = logging.getLogger(__name__)

_SKILL_FILE = Path(__file__).parent / "skills" / "pick.md"

PICK_TOOLS = {
    "perception__segment_objects",
    "perception__get_grasp_from_pointcloud",
    "moveit__plan_and_execute",
    "moveit__get_current_pose",
    "moveit__clear_planning_scene",
    "ros__call_service",
    "ros__send_action_goal",
    "ros__subscribe_once",
}

CREEP_TOOL = {
    "type": "function",
    "function": {
        "name": "creep_closer",
        "description": (
            "Drive the base forward to bring the grasp target into arm reach. "
            "Use ONLY when get_grasp_from_pointcloud returns x > 0.95m. "
            "Reads /amcl_pose, drives nav2 forward by (current_grasp_x - 0.65)m, "
            "clears the octomap, re-segments on the arm camera, and recomputes "
            "the grasp pose. Returns the new segmentation + grasp output. "
            "Call AT MOST 3 times per pick. If grasp_x is still > 0.95m after "
            "the second creep, report FAILURE."
        ),
        "parameters": {
            "type": "object",
            "properties": {
                "object_name": {
                    "type": "string",
                    "description": "Same prompt passed to segment_objects.",
                },
                "current_grasp_x": {
                    "type": "number",
                    "description": "x coordinate of the latest grasp pose in base_footprint (the value > 0.85m that triggered the creep).",
                },
            },
            "required": ["object_name", "current_grasp_x"],
        },
    },
}


def _filter_tools(all_tools: list[dict]) -> list[dict]:
    return [t for t in all_tools if t["function"]["name"] in PICK_TOOLS]


async def _creep_closer(
    mcp: MCPClient,
    object_name: str,
    current_grasp_x: float,
) -> str:
    """Read amcl_pose, drive forward, re-segment, re-grasp.

    Mirrors the navigator's _approach_forward pattern (navigator.py:212-239)
    so the pick LLM never needs to touch nav2 directly.
    """
    pose_raw = await mcp.call_tool_prefixed(
        "ros__subscribe_once",
        {
            "topic": "/amcl_pose",
            "msg_type": "geometry_msgs/msg/PoseWithCovarianceStamped",
            "timeout": 3,
        },
    )
    pose_data = json.loads(pose_raw) if isinstance(pose_raw, str) else pose_raw
    if "msg" in pose_data:
        pose_data = pose_data["msg"]
    pos = pose_data["pose"]["pose"]["position"]
    ori = pose_data["pose"]["pose"]["orientation"]
    siny = 2.0 * (ori["w"] * ori["z"] + ori["x"] * ori["y"])
    cosy = 1.0 - 2.0 * (ori["y"] ** 2 + ori["z"] ** 2)
    yaw = math.atan2(siny, cosy)

    d = current_grasp_x - 0.65
    goal_x = pos["x"] + math.cos(yaw) * d
    goal_y = pos["y"] + math.sin(yaw) * d

    logger.info(
        f"  [creep] grasp_x={current_grasp_x:.2f} d={d:.2f} → "
        f"({pos['x']:.2f},{pos['y']:.2f}) → ({goal_x:.2f},{goal_y:.2f})"
    )

    await mcp.call_tool_prefixed(
        "nav2__navigate_to_pose",
        {"x": round(goal_x, 2), "y": round(goal_y, 2), "yaw": round(yaw, 2)},
    )
    await mcp.call_tool_prefixed(
        "ros__call_service",
        {
            "service_name": "/clear_octomap",
            "service_type": "std_srvs/srv/Empty",
            "request": {},
        },
    )
    seg = await mcp.call_tool_prefixed(
        "perception__segment_objects",
        {"prompt": object_name, "camera": "arm"},
    )
    grasp = await mcp.call_tool_prefixed(
        "perception__get_grasp_from_pointcloud",
        {"object_name": object_name},
    )
    return (
        f"creep_closer: drove {d:.2f}m forward, re-segmented on arm camera, "
        f"recomputed grasp.\n--- segmentation ---\n{seg}\n--- grasp ---\n{grasp}"
    )


async def execute_pick(
    mcp: MCPClient,
    object_name: str,
    model: str = None,
    max_tool_calls: int = 30,
) -> dict:
    """Run a pick agent to grasp an object.

    The robot must already be positioned roughly in front of the object
    (navigator handoff). The agent may invoke the `creep_closer` helper
    up to twice if the initial grasp is just out of arm reach.

    Args:
        mcp: Connected MCPClient instance.
        object_name: Name of the object to pick (e.g. "red ball").
        model: LiteLLM model string. Defaults to LLM_MODEL env var.
        max_tool_calls: Safety cap on tool calls to prevent runaway.

    Returns:
        {"success": bool, "reason": str, "tool_calls_used": int}
    """
    all_tools = mcp.get_tools()
    tools = _filter_tools(all_tools) + [CREEP_TOOL]

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
            messages.append(assistant_message(response))

            tool_results = []
            for tc_id, tc_name, tc_args in get_tool_calls(response):
                tool_call_count += 1
                logger.info(
                    f"  [{tool_call_count}] {tc_name}({json.dumps(tc_args)[:200]})"
                )

                try:
                    if tc_name == "creep_closer":
                        result = await _creep_closer(mcp, **tc_args)
                    else:
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
            logger.info(f"Pick finished: {final_text[:200]}")

            # Find last standalone SUCCESS/FAILURE; whichever appears later wins
            s_matches = list(re.finditer(r'\bSUCCESS\b', final_text, re.IGNORECASE))
            f_matches = list(re.finditer(r'\bFAILURE\b', final_text, re.IGNORECASE))
            last_s = s_matches[-1].start() if s_matches else -1
            last_f = f_matches[-1].start() if f_matches else -1
            success = last_s > last_f
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

    logger.warning(f"Pick hit max tool calls ({max_tool_calls})")
    return {
        "success": False,
        "reason": f"Exceeded maximum tool calls ({max_tool_calls})",
        "tool_calls_used": tool_call_count,
    }
