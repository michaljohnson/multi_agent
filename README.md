# Multi-Agent Long-Horizon Manipulation

A four-agent system for autonomous pick-and-place tasks on a Summit XL mobile manipulator (UR5 + Robotiq 2F-140) using ROS 2 Jazzy. Built with [LiteLLM](https://github.com/BerriAI/litellm) (provider-agnostic LLM client) and [MCP](https://modelcontextprotocol.io/) (Model Context Protocol) for robot control.

**Bachelor thesis context:** comparison of three architectures (single-agent, multi-agent, deterministic skill-based) for long-horizon agentic manipulation. This module is the multi-agent track.

## Architecture

![Multi-agent architecture](docs/architecture.png)

The system is a three-tier agentic stack:

- **High level — Orchestrator (full autonomy).** Receives a natural-language task, decomposes it into `navigate(...)`, `pick(...)`, `place(...)` sub-agent calls, and uses `look()` for visual ground-truth checks between steps. Reasoning-only — never touches ROS directly.
- **Middle level — Sub-agents.** Three LLM agents (Navigator, Pick, Place) each with a markdown skill prompt and a filtered MCP tool set. Navigator includes a deterministic spin-search helper; Pick and Place share a deterministic creeper helper.
- **Low level — MCP client + 4 MCP servers.** All ROS interaction goes through MCP (`ros_mcp_server`, `moveit_mcp_server`, `nav2_mcp_server`, `perception_mcp_server`). No Python script touches ROS topics or actions directly.

LLM access goes through a single LiteLLM client → any provider (Anthropic, OpenAI, local Ollama, OpenAI-compatible endpoints).

Each agent is an LLM (default `claude-sonnet-4-20250514`) with a markdown **skill prompt** in `skills/` that defines its procedure, plus a filtered set of MCP tools.

- **Orchestrator** (`orchestrator.py`, `skills/orchestrator.md`) — owns task decomposition. Calls navigate/pick/place sub-agents. Reasoning-only; does NOT touch ROS directly.
- **Navigator** (`navigator.py`, `skills/navigator.md`) — drives to a known map area, verifies arrival via `look()` + LLM vision + SAM3 target check on either camera. Includes a deterministic spin-search fallback when the target is missing.
- **Pick** (`pick.py`, `skills/pick.md`) — segment → grasp pose → reach check → optional `creep_closer` (drives base into reach via nav2) → pre-grasp → lower → close → verify via `/gripper/status`.
- **Place** (`place.py`, `skills/place.md`) — three placement modes:
  - **Floor** — fast path, no segmentation, soft contact (object-height-aware descent)
  - **Surface** — segment-from-above (arm cam at `look_down_high`) for accurate top-z, then soft contact
  - **Container** — full two-stage (front-cam coarse → arm-cam refine) with creep

## Scope (multi-agent track)

**In scope:** floor pickup (cube, ball, shoe, can on floor) + low coffee table (~0.45m) pickup, floor placement, container placement (trash bin, drainer).

**Out of scope:** higher surfaces (kitchen counter, dining table). These are explicitly left to the skill-based architecture for thesis comparison — multi-agent's lack of cross-step reasoning makes height-dependent standoff and complex recovery brittle.

## Prerequisites

- ROS 2 Jazzy + Summit XL simulation up (Gazebo, MoveIt, Nav2 stack, SAM3 segmentation nodes for arm + front cameras)
- Rosbridge WebSocket server on port 9090
- 4 MCP servers running and reachable (default ports below):
  - `ros-mcp-server` on port 8888
  - `moveit-mcp-server` on port 8001
  - `nav2-mcp-server` on port 8002
  - `perception-mcp-server` on port 8003

Each MCP server is its own repository. Install + run them separately. Connection URLs are configured in your `.mcp.json`.

## Setup

1. Install dependencies:
   ```bash
   pip install litellm mcp --break-system-packages
   ```

2. Configure environment:
   ```bash
   cp .env.example .env
   # Edit .env: set ANTHROPIC_API_KEY (or OPENAI_API_KEY, or any LiteLLM-supported provider)
   ```

3. Source env vars before running:
   ```bash
   set -a && source .env && set +a
   ```

## Usage

### Full orchestrator (long-horizon task)

```bash
python3 -u -m multi_agent.main --task "Pick up the white cube in the kids room and drop it in the trash bin"
```

### Test individual agents (skip orchestrator)

```bash
# pick agent — assumes robot is already near the target
python3 -m multi_agent.main --test-pick "white cube"

# place agent — assumes robot is holding an object near the target
python3 -m multi_agent.main --test-place "trash bin"
python3 -m multi_agent.main --test-place "the floor in front of you"   # floor mode

# navigator agent — drives to area + verifies arrival
python3 -m multi_agent.main --test-navigator "kids room" --target-object "white cube"
```

### Verbose logging (debugging)

```bash
python3 -u -m multi_agent.main --task "..." --verbose
```

Default mode strips timestamps and framework noise — agent traces read like a narrative of decisions and tool calls. `--verbose` adds full timestamps + LiteLLM/MCP framework lines for debugging.

## LLM provider

Default is Anthropic Claude. To switch:

```bash
export LLM_MODEL=gpt-4o                                # OpenAI
export LLM_MODEL=ollama/llama3                         # local Ollama
export LLM_MODEL=anthropic/claude-sonnet-4-20250514    # Anthropic (default)
```

Or per-role:
```bash
export ORCHESTRATOR_MODEL=anthropic/claude-opus-4
export EXECUTOR_MODEL=anthropic/claude-sonnet-4
export NAVIGATOR_MODEL=anthropic/claude-sonnet-4
```

## Project structure

```
multi_agent/
├── main.py                # Entry point, CLI, logging config
├── orchestrator.py        # Top-level planner — calls navigate/pick/place
├── navigator.py           # Navigation agent + post-step (verify gate, spin-search)
├── pick.py                # Pick agent + creep_closer helper
├── place.py               # Place agent + creep_closer helper + verify_object_placed
├── llm_client.py          # LiteLLM wrapper (provider-agnostic)
├── mcp_client.py          # Async MCP server connection manager
├── skills/
│   ├── orchestrator.md
│   ├── navigator.md       # Map / entry poses / pickable + placeable tables
│   ├── pick.md
│   └── place.md
├── docs/                  # Architecture diagrams + thesis-related docs
├── .env.example
└── README.md
```

## Map / environment knowledge

The navigator's environment knowledge (room layout, entry poses, pickable objects, place targets) lives in `skills/navigator.md`. Update that file when:
- Adding/moving objects in the world
- Adjusting room layout or entry poses
- Discovering better SAM3 prompts for an object class

This is intentional: map knowledge is operational deployment configuration (any robot needs it). It is NOT hardcoded sim behavior — the same skills file works on a real Summit XL with the same map.

## Real-world deployability

Code in this module is designed to deploy to a real Summit XL after the thesis. No `gz model` calls or Gazebo plugin usage in production logic. Only validated, real-world-compatible primitives:
- TF lookups via `nav2__get_robot_pose` (live `tf2_ros` listener with dedicated executor)
- nav2 actions (`navigate_to_pose`, `DriveOnHeading`, `spin_robot`) for base motion
- MoveIt actions (`plan_and_execute`) for arm motion
- SAM3-based segmentation for perception
- Standard `/gripper/status` for grasp verification

Sim-only references (model names, world coordinates) are confined to the skill prompts (`skills/navigator.md`) — they are operational configuration, not application code.

## Known limitations

- **nav2_mcp_server zombie goal** — when an MCP timeout fires, the server-side action goal stays active. Robot may execute the goal autonomously minutes later. Workaround: full sim restart on timeout.
- **Gripper force-detach DDS race** — `/gripper/force_detach_str` publish occasionally fails silently. Workaround: verify via `/gripper/status` and retry.
- **Verify-gate false positives on surface placement** — the front-cam check "object no longer visible" passes when an object falls behind / under furniture, looking identical to a real successful container drop. Reliable surface placement requires arm-cam-from-above verification.
