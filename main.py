#!/usr/bin/env python3
"""Multi-agent room cleaning system.

Usage:
    python3 -m multi_agent.main --task "pick up the screwdriver from the coffee table and bring it to the kitchen table"
    python3 -m multi_agent.main --test-pick "screwdriver"        
    python3 -m multi_agent.main --test-navigator "kitchen" --target-object "red coke can"
    python3 -m multi_agent.main --test-place "trashbin"  
"""

import argparse
import asyncio
import json
import logging
import os

from multi_agent.mcp_client import MCPClient
from multi_agent.pick import execute_pick
from multi_agent.place import execute_place
from multi_agent.orchestrator import run_orchestrator

# === CONFIGURATION ===

# LLM model — provider-agnostic via LiteLLM. Set LLM_MODEL in your .env
# to any model id LiteLLM recognises, e.g.:
#   LLM_MODEL=gpt-4o                                # OpenAI
#   LLM_MODEL=ollama/llama3                         # local Ollama
#   LLM_MODEL=anthropic/claude-sonnet-4-20250514    # Anthropic
#   LLM_MODEL=openai/qwen3-vl-...                   # any OpenAI-compatible
# Per-role overrides: ORCHESTRATOR_MODEL, EXECUTOR_MODEL, NAVIGATOR_MODEL.
# Whatever provider is selected, ensure its native API key env var is
# exported (e.g. ANTHROPIC_API_KEY, OPENAI_API_KEY) — LiteLLM picks them
# up automatically.
LLM_MODEL = os.environ.get("LLM_MODEL", "anthropic/claude-sonnet-4-20250514")
ORCHESTRATOR_MODEL = os.environ.get("ORCHESTRATOR_MODEL", LLM_MODEL)
EXECUTOR_MODEL = os.environ.get("EXECUTOR_MODEL", LLM_MODEL)
NAVIGATOR_MODEL = os.environ.get("NAVIGATOR_MODEL", LLM_MODEL)

async def test_pick(object_name: str):
    """Test pick executor on a single object (no orchestrator/navigator).

    Assumes the robot is already positioned in front of the target object.
    """
    print(f"\n=== Testing pick executor: '{object_name}' ===\n")

    async with MCPClient() as mcp:
        result = await execute_pick(
            mcp=mcp,
            object_name=object_name,
            model=EXECUTOR_MODEL,
        )
        print(f"\n=== Pick Result ===")
        print(json.dumps(result, indent=2))


async def test_place(target_container: str):
    """Test place executor on a single target (no orchestrator/navigator).

    Assumes the robot is already holding an object and positioned near
    the drop target.
    """
    print(f"\n=== Testing place executor: target='{target_container}' ===\n")

    async with MCPClient() as mcp:
        result = await execute_place(
            mcp=mcp,
            target_container=target_container,
            model=EXECUTOR_MODEL,
        )
        print(f"\n=== Place Result ===")
        print(json.dumps(result, indent=2))


async def test_navigator(
    destination: str,
    target_object: str | None = None,
):
    """Test the navigator with a natural language destination (no orchestrator)."""
    from multi_agent.navigator import execute_navigate

    print(f"\n=== Testing navigator: '{destination}' ===")
    if target_object:
        print(f"    Target object: '{target_object}'")
    print()

    async with MCPClient() as mcp:
        result = await execute_navigate(
            mcp=mcp,
            destination=destination,
            target_object=target_object,
            model=NAVIGATOR_MODEL,
        )
        print(f"\n=== Result ===")
        print(json.dumps(result, indent=2))


async def run_full(task: str):
    """Run the full orchestrator with an open-ended task."""
    print(f"\n=== Task: {task} ===")
    print(f"Orchestrator: {ORCHESTRATOR_MODEL}")
    print(f"Navigator:    {NAVIGATOR_MODEL}")
    print(f"Executor:     {EXECUTOR_MODEL}\n")

    async with MCPClient() as mcp:
        result = await run_orchestrator(
            mcp=mcp,
            task=task,
            orchestrator_model=ORCHESTRATOR_MODEL,
            executor_model=EXECUTOR_MODEL,
            navigator_model=NAVIGATOR_MODEL,
        )
        print(f"\n=== Final Report ===")
        print(result["summary"])
        print(f"\nOrchestrator turns used: {result['turns_used']}")


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
        help="Test single pick of OBJECT (no orchestrator/navigator)",
    )
    parser.add_argument(
        "--test-place",
        type=str,
        metavar="CONTAINER",
        default=None,
        help="Test single place into CONTAINER (no orchestrator/navigator). Robot must already be holding an object.",
    )
    parser.add_argument(
        "--test-navigator",
        type=str,
        nargs="+",
        metavar="DEST",
        default=None,
        help="Test navigation to DEST, optionally with --target-object (e.g. --test-navigator the kitchen area)",
    )
    parser.add_argument(
        "--target-object",
        type=str,
        default=None,
        help="Target object context for navigator (e.g. 'red coke can on the floor')",
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Enable verbose logging",
    )
    args = parser.parse_args()

    level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
        datefmt="%H:%M:%S",
    )
    logging.getLogger("httpx").setLevel(logging.WARNING)
    logging.getLogger("mcp").setLevel(logging.WARNING)

    if args.test_pick:
        asyncio.run(test_pick(args.test_pick))
    elif args.test_place:
        asyncio.run(test_place(args.test_place))
    elif args.test_navigator:
        dest = " ".join(args.test_navigator)
        asyncio.run(test_navigator(dest, args.target_object))
    elif args.task:
        asyncio.run(run_full(args.task))
    else:
        parser.error(
            "no action specified — pass --task, --test-pick, --test-place, "
            "or --test-navigator"
        )


if __name__ == "__main__":
    main()
