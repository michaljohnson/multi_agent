#!/usr/bin/env python3
"""Multi-agent room cleaning system.

Usage:
    python3 -m multi_agent.main --task "pick up the white cube in the kids room and place it into the trash bin in the kids room"
    python3 -m multi_agent.main --test-pick "white cube"        
    python3 -m multi_agent.main --test-approach "kitchen" --object-name "red coke can"
    python3 -m multi_agent.main --test-place "trashbin"  
"""

import argparse
import asyncio
import json
import logging
import os
import time
from pathlib import Path

from dotenv import load_dotenv

load_dotenv(Path(__file__).parent / ".env")

from multi_agent.clients.mcp import MCPClient
from multi_agent.subagents.pick import execute_pick
from multi_agent.subagents.place import execute_place
from multi_agent.orchestrator import run_orchestrator

# === CONFIGURATION ===

# LLM model — provider-agnostic via LiteLLM. Required env var. Set
# LLM_MODEL in your .env to any model id LiteLLM recognises, e.g.:
#   LLM_MODEL=gpt-4o                                # OpenAI
#   LLM_MODEL=ollama/llama3                         # local Ollama
#   LLM_MODEL=anthropic/claude-sonnet-4-20250514    # Anthropic
#   LLM_MODEL=openai/qwen3-vl-...                   # any OpenAI-compatible
# Per-role overrides: ORCHESTRATOR_MODEL, EXECUTOR_MODEL, APPROACH_MODEL.
# Whatever provider is selected, ensure its native API key env var is
# exported (e.g. ANTHROPIC_API_KEY, OPENAI_API_KEY) — LiteLLM picks them
# up automatically. See .env.example for the full menu.
LLM_MODEL = os.environ.get("LLM_MODEL")
if not LLM_MODEL:
    raise SystemExit(
        "LLM_MODEL is not set. Copy multi_agent/.env.example to "
        "multi_agent/.env and uncomment exactly one LLM_MODEL line."
    )
ORCHESTRATOR_MODEL = os.environ.get("ORCHESTRATOR_MODEL", LLM_MODEL)
EXECUTOR_MODEL = os.environ.get("EXECUTOR_MODEL", LLM_MODEL)
APPROACH_MODEL = os.environ.get("APPROACH_MODEL", LLM_MODEL)

async def test_pick(object_name: str):
    """Test pick executor on a single object (no orchestrator/approach).

    Assumes the robot is already positioned in front of the target object.
    """
    print(f"\n=== Testing pick executor: '{object_name}' ===\n")

    async with MCPClient() as mcp:
        t0 = time.perf_counter()
        result = await execute_pick(
            mcp=mcp,
            object_name=object_name,
            model=EXECUTOR_MODEL,
        )
        result["wall_seconds"] = round(time.perf_counter() - t0, 2)
        print(f"\n=== Pick Result ===")
        print(json.dumps(result, indent=2))


async def test_place(target_location: str, object_name: str):
    """Test place executor on a single target (no orchestrator/approach).

    Assumes the robot is already holding an object and positioned near
    the drop target.
    """
    print(f"\n=== Testing place executor: target='{target_location}' ===")
    print(f"    Held object: '{object_name}'")
    print()

    async with MCPClient() as mcp:
        t0 = time.perf_counter()
        result = await execute_place(
            mcp=mcp,
            target_location=target_location,
            object_name=object_name,
            model=EXECUTOR_MODEL,
        )
        result["wall_seconds"] = round(time.perf_counter() - t0, 2)
        print(f"\n=== Place Result ===")
        print(json.dumps(result, indent=2))


async def test_approach(
    target_area: str,
    object_name: str,
    next_action: str,
):
    """Test the approach agent with a natural language target area (no orchestrator)."""
    from multi_agent.subagents.approach import execute_approach

    print(f"\n=== Testing approach: '{target_area}' ===")
    print(f"    Target object: '{object_name}'")
    print(f"    Next action:   '{next_action}'")
    print()

    async with MCPClient() as mcp:
        t0 = time.perf_counter()
        result = await execute_approach(
            mcp=mcp,
            target_area=target_area,
            object_name=object_name,
            next_action=next_action,
            model=APPROACH_MODEL,
        )
        result["wall_seconds"] = round(time.perf_counter() - t0, 2)
        print(f"\n=== Result ===")
        print(json.dumps(result, indent=2))


async def run_full(task: str):
    """Run the full orchestrator with an open-ended task."""
    print(f"\n=== Task: {task} ===")
    print(f"Orchestrator: {ORCHESTRATOR_MODEL}")
    print(f"Approach:     {APPROACH_MODEL}")
    print(f"Pick:         {EXECUTOR_MODEL}")
    print(f"Place:        {EXECUTOR_MODEL}\n")

    async with MCPClient() as mcp:
        t0 = time.perf_counter()
        result = await run_orchestrator(
            mcp=mcp,
            task=task,
            orchestrator_model=ORCHESTRATOR_MODEL,
            executor_model=EXECUTOR_MODEL,
            approach_model=APPROACH_MODEL,
        )
        wall_seconds = round(time.perf_counter() - t0, 2)
        print(f"\n=== Final Report ===")
        print(result["summary"])
        print(f"\nOrchestrator turns used:    {result['turns_used']}")
        print(f"Sub-agent tool calls total: {result['subagent_tool_calls_total']}")
        print(f"Wall-clock total:           {wall_seconds}s ({wall_seconds / 60:.1f} min)")


def main():
    parser = argparse.ArgumentParser(description="Multi-agent room cleaning")
    parser.add_argument(
        "--task",
        type=str,
        default=None,
        help="Natural language task (e.g. 'pick up the clamp and bring it to the kitchen')",
    )
    parser.add_argument(
        "--test-pick",
        type=str,
        metavar="OBJECT",
        default=None,
        help="Test single pick of OBJECT (no orchestrator/approach)",
    )
    parser.add_argument(
        "--test-place",
        type=str,
        metavar="CONTAINER",
        default=None,
        help="Test single place into CONTAINER (no orchestrator/approach). Robot must already be holding an object.",
    )
    parser.add_argument(
        "--test-approach",
        type=str,
        nargs="+",
        metavar="DEST",
        default=None,
        help="Test navigation to DEST, optionally with --object-name (e.g. --test-approach the kitchen area)",
    )
    parser.add_argument(
        "--object-name",
        type=str,
        default=None,
        help="Target object context for approach agent (e.g. 'red coke can on the floor')",
    )
    parser.add_argument(
        "--next-action",
        type=str,
        choices=["pick", "surface_place", "container_place", "floor_place"],
        default=None,
        help=(
            "Required for --test-approach. What the next subagent call "
            "will be — determines the approach standoff. 'pick' / "
            "'floor_place' = 0.85m, 'container_place' = 0.65m, "
            "'surface_place' = 0.45m."
        ),
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Enable verbose logging",
    )
    args = parser.parse_args()

    if args.verbose:
        # Full debug logging with timestamps + logger names + level
        logging.basicConfig(
            level=logging.DEBUG,
            format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
            datefmt="%H:%M:%S",
        )
    else:
        # Demo-friendly: just an agent tag + the message. No timestamps,
        # no levels (unless WARNING+), no LiteLLM / MCP framework noise.
        # Easy to follow what each agent is doing. ANSI colour codes are
        # only emitted to a real terminal (sys.stderr.isatty()); piping
        # the output to a file or via tee keeps the captured log plain.
        import re
        import sys

        _USE_COLOR = sys.stderr.isatty()
        _RED    = "\033[31m" if _USE_COLOR else ""
        _GREEN  = "\033[32m" if _USE_COLOR else ""
        _YELLOW = "\033[33m" if _USE_COLOR else ""
        _BOLD   = "\033[1m"  if _USE_COLOR else ""
        _RESET  = "\033[0m"  if _USE_COLOR else ""

        # Colour rules:
        #   success=true / "success": true / success=True  -> green
        #   success=false / "success": false / success=False -> red
        #   error_code=NONE / "error_code": "NONE"          -> green
        #   error_code=<X>  / "error_code": "<X>" (X!=NONE) -> red
        #   standalone SUCCESS / FAILED / FAILURE / FAIL    -> green / red
        _RE_SUCCESS_TRUE  = re.compile(r'("success"\s*:\s*true|success\s*=\s*[Tt]rue|success=True)')
        _RE_SUCCESS_FALSE = re.compile(r'("success"\s*:\s*false|success\s*=\s*[Ff]alse|success=False)')
        _RE_ERR_NONE      = re.compile(r'("error_code"\s*:\s*"NONE"|error_code=NONE)')
        _RE_ERR_OTHER     = re.compile(r'("error_code"\s*:\s*"(?!NONE")[A-Z_]+"|error_code=(?!NONE\b)[A-Z_]+)')
        _RE_KW_SUCCESS    = re.compile(r'\bSUCCESS\b')
        _RE_KW_FAILED     = re.compile(r'\b(FAILED|FAILURE|FAIL)\b')

        def _colorize(msg: str) -> str:
            if not _USE_COLOR:
                return msg
            msg = _RE_SUCCESS_TRUE.sub(lambda m: _GREEN + m.group(0) + _RESET, msg)
            msg = _RE_SUCCESS_FALSE.sub(lambda m: _RED + m.group(0) + _RESET, msg)
            msg = _RE_ERR_NONE.sub(lambda m: _GREEN + m.group(0) + _RESET, msg)
            msg = _RE_ERR_OTHER.sub(lambda m: _RED + m.group(0) + _RESET, msg)
            msg = _RE_KW_SUCCESS.sub(_GREEN + "SUCCESS" + _RESET, msg)
            msg = _RE_KW_FAILED.sub(lambda m: _RED + m.group(0) + _RESET, msg)
            return msg

        class _AgentTagFormatter(logging.Formatter):
            _TAGS = {
                "multi_agent.orchestrator":         "[ORCHESTRATOR]",
                "multi_agent.subagents.approach":  "[APPROACH]    ",
                "multi_agent.subagents.pick":      "[PICK]        ",
                "multi_agent.subagents.place":     "[PLACE]       ",
                "multi_agent.clients.mcp":         "[MCP]         ",
            }

            def format(self, record: logging.LogRecord) -> str:
                msg = record.getMessage()
                # Section-break lines (e.g. "=== ORCHESTRATOR turn 1/50 ===")
                # are printed without a tag prefix so they stand out as
                # visual boundaries between orchestrator decisions and
                # sub-agent dispatches. Same convention as the skill-based
                # planner's per-decision section breaks. Bold on terminal.
                if msg.startswith("==="):
                    return f"{_BOLD}{msg}{_RESET}"
                tag = self._TAGS.get(record.name, f"[{record.name}]")
                msg = _colorize(msg)
                if record.levelno >= logging.WARNING:
                    level_color = _RED if record.levelno >= logging.ERROR else _YELLOW
                    return f"{tag} {level_color}{record.levelname}{_RESET}: {msg}"
                return f"{tag} {msg}"

        handler = logging.StreamHandler()
        handler.setFormatter(_AgentTagFormatter())
        root = logging.getLogger()
        root.handlers = [handler]
        root.setLevel(logging.INFO)

    # Suppress framework noise regardless of verbosity unless explicitly DEBUG
    framework_level = logging.DEBUG if args.verbose else logging.WARNING
    logging.getLogger("httpx").setLevel(framework_level)
    logging.getLogger("mcp").setLevel(framework_level)
    logging.getLogger("LiteLLM").setLevel(framework_level)
    logging.getLogger("litellm").setLevel(framework_level)
    logging.getLogger("multi_agent.clients.mcp").setLevel(framework_level)

    if args.test_pick:
        asyncio.run(test_pick(args.test_pick))
    elif args.test_place:
        if not args.object_name:
            parser.error(
                "--test-place requires --object-name (step 12's look-down "
                "vision verify needs a label to decide whether the released "
                "object landed in/on the target)"
            )
        asyncio.run(test_place(args.test_place, args.object_name))
    elif args.test_approach:
        area = " ".join(args.test_approach)
        if not args.object_name:
            parser.error(
                "--test-approach requires --object-name (the approach agent's "
                "job is to put the target in view; without one it has nothing "
                "to verify)"
            )
        if not args.next_action:
            parser.error(
                "--test-approach requires --next-action (picks the approach "
                "standoff: 'pick' / 'floor_place' = 0.85m, "
                "'container_place' = 0.65m, 'surface_place' = 0.45m). "
                "The silent default to 'pick' previously broke downstream "
                "places that needed a closer standoff."
            )
        asyncio.run(test_approach(area, args.object_name, args.next_action))
    elif args.task:
        asyncio.run(run_full(args.task))
    else:
        parser.error(
            "no action specified — pass --task, --test-pick, --test-place, "
            "or --test-approach"
        )


if __name__ == "__main__":
    main()
