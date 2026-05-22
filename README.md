# Multi-agent architecture
![License](https://img.shields.io/badge/license-Apache--2.0-blue)
![Python](https://img.shields.io/badge/python-3.10%2B-blue)
![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314e)
![LiteLLM](https://img.shields.io/badge/LiteLLM-provider--agnostic-7e3fa8)
![MCP](https://img.shields.io/badge/MCP-client-orange)
![Last commit](https://img.shields.io/github/last-commit/michaljohnson/multi_agent)

A four-agent system for autonomous pick-and-place tasks on a [Summit XL](https://github.com/icclab/icclab_summit_xl) mobile manipulator (UR5 + Robotiq 2F-140) using ROS 2 Jazzy. Built with [LiteLLM](https://github.com/BerriAI/litellm) (provider-agnostic LLM client) and [MCP](https://modelcontextprotocol.io/) (Model Context Protocol) for robot control.


![Multi-agent architecture](docs/architecture.png)

The system is a three-tier agentic stack: an orchestrator LLM decomposes a natural-language task into sub-agent calls; each sub-agent is a separate LLM with a narrow MCP tool set and its own system prompt; the MCP layer below talks to the robot.

## Short demo

https://github.com/user-attachments/assets/883cf776-8e0a-4655-bed2-2917f652394b


Originally built as one of three architectures compared in a BA thesis on "where the policy should live" in agentic robotics; released so others can reuse the pattern.

## Comparison-axis position 

| Architecture | Where the policy lives | Planner LLM |
|---|---|---|
| [Single-agent](https://github.com/anthropics/claude-code) | LLM context, raw MCP tool surface | Frontier (e.g. Claude Opus) |
| **Multi-agent** | **Orchestrator + 3 LLM sub-agents, narrow MCP subsets per agent** | **Frontier (e.g. Claude Opus)** |
| [Skill-based](https://github.com/michaljohnson/skill_based) | Inside Python skills, hidden from the LLM | Small open-weights or frontier |

Multi-agent's design point: each LLM sees only the tools it needs (3-7 each), not the full ~60 across all four MCP servers. Decomposition narrows the per-call decision space at the cost of cross-agent coordination overhead.

## Requirements

- Python 3.10+
- `litellm`, `python-dotenv`, an `mcp` Python client (any FastMCP-style streamable-http client works)
- One or more MCP servers exposing the tool families this code expects:
  - [**nav2**](https://github.com/michaljohnson/nav2_mcp_server) — drive, navigate-to-pose, spin, lifecycle, `approach_target`
  - [**moveit**](https://github.com/michaljohnson/moveit-mcp-server) — plan/execute, IK, planning scene
  - [**perception**](https://github.com/michaljohnson/perception_mcp_server) — segmentation, top-down grasp/place pose, look
  - [**ros**](https://github.com/robotmcp/ros-mcp-server) — generic topics, services, actions, parameters
- An LLM provider (Anthropic API, OpenAI-compatible vLLM, OpenAI, Ollama — anything LiteLLM supports)

The MCP servers are not part of this package; you bring your own. The architecture's contract with them is "any tool-calling LLM should be able to use them," which is exactly what an MCP server provides.

## Repository layout

```
multi_agent/
  main.py                  CLI entry (--task / --test-{pick,place,approach})
  orchestrator.py          top-level brain — decomposes task into approach/pick/place sub-agent calls
  orchestrator.md          orchestrator system prompt (loaded by orchestrator.py)
  subagents/               LLM-driven sub-agents, each with its own MCP tool subset and prompt
    __init__.py
    approach.py / .md      drives base + finds target via LLM-vision + spin-search + drives to standoff
    pick.py / .md          arm-cam segment → grasp pose → pre-grasp → descend → close → verify
    place.py / .md         front-cam segment → drop pose → pre-place → descend → release → verify
  clients/                 external system adapters
    __init__.py
    llm.py                 LiteLLM wrapper (provider-agnostic)
    mcp.py                 MCP connection manager
  docs/
  .env.example             environment-variable template
  README.md                this file
```

## Quick start

```bash
cp multi_agent/.env.example multi_agent/.env
# edit .env: pick a planner LLM (Anthropic, OpenAI-compatible local vLLM, OpenAI, ...)

# Single-skill smoke tests (assume robot is pre-positioned for pick/place):
python3 -m multi_agent.main --test-pick "red coke can"
python3 -m multi_agent.main --test-place "trash bin" --object-name "red coke can" --object-height-m 0.12
python3 -m multi_agent.main --test-approach "living room" --object-name "wooden coffee table" --next-action pick 

# Full orchestrator (long-horizon task):
python3 -m multi_agent.main --task "pick up the red coke can in the kitchen and place it on the wooden coffee table in the living room"
```

`.env` is loaded automatically via `python-dotenv` from the package root. Run from the parent directory of `multi_agent/` so the package import resolves.

`--verbose` adds full timestamps + LiteLLM/MCP framework lines for debugging. Default mode strips timestamps and framework noise — agent traces read like a narrative of decisions and tool calls.

## How a turn works

1. Orchestrator LLM receives the user task and three sub-agent tool schemas (`approach`, `pick`, `place`) plus `look()` for visual ground-truth checks.
2. Orchestrator emits a sub-agent call (e.g. `approach(target_area="kitchen", object_name="red coke can", next_action="pick")`).
3. The matching sub-agent in `subagents/` is invoked — it loads its own `.md` system prompt, sees its own narrow MCP tool subset (3-7 tools), and runs its own LLM loop.
4. The sub-agent returns a structured result `{success: bool, error_code: str, reason: str, tool_calls_used: int, ...}` to the orchestrator.
5. The orchestrator reads the result, optionally calls `look()` for visual confirmation, decides the next sub-agent or returns a final report.

Each sub-agent has its own LLM context — the orchestrator does NOT see the sub-agents' internal MCP traces. This is the design point: each layer reasons over a narrower decision space than the layer above.

## Typed error_code contract

Every sub-agent terminates by calling its `report_*_result` tool with a typed `error_code` from a fixed enum (plus a prose `reason` for context). The orchestrator routes recovery decisions on the enum and reads the prose only for the long tail. This is the architectural contrast versus the skill-based sibling, which returns prose-only results.

| Sub-agent | `error_code` values |
|---|---|
| approach | `NONE`, `NAV_PLAN_FAILED`, `NAV_DRIVE_FAILED`, `NAV_AREA_WRONG`, `NAV_TARGET_NOT_VISIBLE`, `NAV_VERIFY_OVERRIDE`, `UNKNOWN` |
| pick | `NONE`, `PICK_SEG_MISSED`, `PICK_PLAN_FAILED`, `PICK_REACH_EXCEEDED`, `PICK_ATTACH_TIMEOUT`, `PICK_HOLDING_ALREADY`, `PICK_WRONG_OBJECT`, `UNKNOWN` |
| place | `NONE`, `PLACE_SEG_MISSED`, `PLACE_PLAN_FAILED`, `PLACE_REACH_EXCEEDED`, `PLACE_DROP_VERIFY_FAILED`, `PLACE_HOLDING_NOTHING`, `PLACE_OUT_OF_SCOPE` |

`NONE` means success. Anything else is a structured failure the orchestrator can branch on (retry the same sub-agent, dispatch a different one, or escalate to the user). The prose `reason` carries the specifics — coordinates, retry counts, perception confidence — that the enum doesn't capture.

## Adapting to your environment

The skills assume specific MCP tool names (e.g. `perception__segment_objects`, `nav2__approach_target`, `moveit__plan_and_execute`). If your MCP servers expose different names, edit the `*_TOOLS` whitelist sets at the top of each `subagents/*.py` and the calls inside. The architectural pattern (orchestrator → sub-agent → MCP tool) is independent of the specific tool names.

The hardcoded entry-pose table in `subagents/approach.md` is keyed to a particular simulated home environment. Replace with your own room/area coordinates.

## Design decisions

- **Each sub-agent has its own LLM and prompt.** The narrow context per agent (one task, 3-7 tools, one system prompt) is the architecture's bet: smaller per-call decision space → better tool selection accuracy. The cost is composition fidelity — sub-agents can't share intermediate context with each other.
- **Orchestrator is reasoning-only.** It never calls MCP tools directly. It only decides which sub-agent to call next, and verifies progress via `look()`. This separates task-decomposition reasoning from low-level robot control.
- **Sub-agent verification is per-skill, not orchestrator-level.** Pick verifies via `/gripper/status`. Place verifies via post-release object visibility. The orchestrator trusts the structured success/failure result. This works for many cases but lets some false-success cases through — see code comments in `subagents/pick.py` and `subagents/place.py`.
- **Failures bubble up; the orchestrator decides retry vs escalate.** Sub-agents do not loop indefinitely on tool errors. Each returns a structured failure that the orchestrator can act on.

## Citing

If you build on this work, please cite the BA thesis it originated from (forthcoming).

## License

See [LICENSE](LICENSE) for details.

## Contributing

Contributions welcome — please open an issue or PR on GitHub.
