#!/usr/bin/env python3
"""Multi-agent room cleaning system.

Usage:
    python -m multi_agent.main                  # Run full orchestrator
    python -m multi_agent.main --test-executor  # Test single pick-and-place
"""

import argparse
import asyncio
import json
import logging
import os

from multi_agent.mcp_client import MCPClient
from multi_agent.executor import execute_pick_and_place
from multi_agent.orchestrator import run_orchestrator

# === CONFIGURATION ===

# LLM model — set LLM_MODEL env var to switch providers, e.g.:
#   export LLM_MODEL=gpt-4o              (OpenAI)
#   export LLM_MODEL=ollama/llama3       (local Ollama)
#   export LLM_MODEL=anthropic/claude-sonnet-4-20250514  (Anthropic, default)
# Per-role overrides: ORCHESTRATOR_MODEL, EXECUTOR_MODEL, NAVIGATOR_MODEL
LLM_MODEL = os.environ.get("LLM_MODEL", "anthropic/claude-sonnet-4-20250514")
ORCHESTRATOR_MODEL = os.environ.get("ORCHESTRATOR_MODEL", LLM_MODEL)
EXECUTOR_MODEL = os.environ.get("EXECUTOR_MODEL", LLM_MODEL)
NAVIGATOR_MODEL = os.environ.get("NAVIGATOR_MODEL", LLM_MODEL)

# Approximate approach poses for known locations (map frame, yaw in radians).
# The orchestrator owns the map; the navigator is a narrow skill that drives
# directly to these and verifies by target object. Measured once via
# get_robot_pose while the robot was manually positioned at each location.
LOCATION_MAP = {
    # Wooden table in the living room area (vision model calls it "wooden desk").
    # Recorded 2026-04-11 by manually driving the robot in front of the table
    # and reading /amcl_pose directly (NOT via nav2-mcp get_robot_pose, which
    # is broken and returns stale data — see project_nav_pose_stale_bug).
    # Cross-validated against Gazebo world pose (delta < 5cm, within AMCL noise).
    # Confirmed by describe_scene: sees living_room, white sofa, wooden table.
    "coffee_table": (1.364, -0.785, -1.645),
    # Wooden dining table in the kitchen/dining area.
    # Recorded 2026-04-11 by manually driving the robot to the table edge
    # and reading /amcl_pose directly. Initially (5.087, 0.962, 0.162) but
    # backed off ~40cm in x because forward_looking carries held objects at
    # EE z≈0.64m — BELOW the 0.75m dining table surface. At the original
    # pose, the held clamp snagged the table edge during approach and pushed
    # the table out of place, causing the place operation to drop the clamp
    # onto the floor (test run 2026-04-11 15:05).
    "kitchen_table": (4.7, 0.962, 0.162),
}

# Task list: natural language descriptions — the orchestrator looks up poses
# in LOCATION_MAP and passes them to the navigator as approach_pose hints.
#
# Two-location test: pick clamp from the living room coffee_table, place it
# on the kitchen dining table. Destination strings use vocabulary the vision
# model actually produces (verified via describe_scene during calibration).
TASK_LIST = [
    {
        "object": "clamp",
        "pickup_location": "the wooden table in the living room area, near the white couch",
        "place_location": "the wooden dining table with navy blue chairs",
        "surface_height": 0.75,  # kitchen_table estimated height
    },
]


async def test_executor():
    """Test pick + place executor on the same surface (no orchestrator/navigator).

    Assumes the robot is already positioned in front of the wooden coffee table
    with the clamp visible. Picks the clamp, then places it back on the same
    surface (coffee table height estimated at 0.45m).
    """
    task = TASK_LIST[0]
    coffee_table_height = 0.45  # estimate — no constant in the sim config
    print(f"\n=== Testing pick+place executor: '{task['object']}' ===\n")

    async with MCPClient() as mcp:
        print("--- PICK ---")
        pick_result = await execute_pick_and_place(
            mcp=mcp,
            object_name=task["object"],
            mode="pick",
            model=EXECUTOR_MODEL,
        )
        print(f"\n=== Pick Result ===")
        print(json.dumps(pick_result, indent=2))

        if not pick_result["success"]:
            print("\nPick failed — skipping place.")
            return

        print("\n--- PLACE (back on same table) ---")
        place_result = await execute_pick_and_place(
            mcp=mcp,
            object_name=task["object"],
            mode="place",
            surface_height=coffee_table_height,
            model=EXECUTOR_MODEL,
        )
        print(f"\n=== Place Result ===")
        print(json.dumps(place_result, indent=2))


async def test_navigator():
    """Test the navigator in guided mode with a single destination.

    Passes approach_pose from LOCATION_MAP so the navigator drives directly
    to a known coordinate and verifies by target_object — no exploration.
    This tests the bounded-skill architecture.
    """
    from multi_agent.navigator import execute_navigate

    task = TASK_LIST[0]
    destination = task["pickup_location"]
    target_object = task["object"]
    approach_pose = LOCATION_MAP["coffee_table"]
    print(
        f"\n=== Testing navigator (guided): target='{target_object}', "
        f"approach_pose={approach_pose} ===\n"
    )

    async with MCPClient() as mcp:
        result = await execute_navigate(
            mcp=mcp,
            destination=destination,
            target_object=target_object,
            approach_pose=approach_pose,
            model=NAVIGATOR_MODEL,
        )
        print(f"\n=== Result ===")
        print(json.dumps(result, indent=2))


async def run_full():
    """Run the full orchestrator with the task list."""
    print(f"\n=== Room Cleaning: {len(TASK_LIST)} objects ===")
    print(f"Orchestrator: {ORCHESTRATOR_MODEL}")
    print(f"Navigator:    {NAVIGATOR_MODEL}")
    print(f"Executor:     {EXECUTOR_MODEL}\n")

    async with MCPClient() as mcp:
        result = await run_orchestrator(
            mcp=mcp,
            task_list=TASK_LIST,
            location_map=LOCATION_MAP,
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
        "--test-executor",
        action="store_true",
        help="Test single pick (no orchestrator/navigator)",
    )
    parser.add_argument(
        "--test-navigator",
        action="store_true",
        help="Test single navigation (no orchestrator)",
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
    # Quiet down httpx
    logging.getLogger("httpx").setLevel(logging.WARNING)
    logging.getLogger("mcp").setLevel(logging.WARNING)

    if args.test_executor:
        asyncio.run(test_executor())
    elif args.test_navigator:
        asyncio.run(test_navigator())
    else:
        asyncio.run(run_full())


if __name__ == "__main__":
    main()
