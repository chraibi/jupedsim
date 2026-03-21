#!/usr/bin/env python3
"""Run a Crossing_big simulation from config.json using scenario.py.

Usage:
    python run_simulation.py                          # default: CollisionFreeSpeedModel
    python run_simulation.py --model WarpDriverModel
    python run_simulation.py --model SocialForceModel
"""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

from scenario import load_scenario, run_scenario

SCRIPT_DIR = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description="Run Crossing_big simulation")
    parser.add_argument(
        "--model", default="CollisionFreeSpeedModel",
        help="Model type (default: CollisionFreeSpeedModel)",
    )
    parser.add_argument("--seed", type=int, default=None, help="Random seed")
    args = parser.parse_args()

    scenario = load_scenario(str(SCRIPT_DIR))

    if args.model != scenario.model_type:
        scenario.set_model_type(args.model)

    print(scenario.summary())
    print()

    result = run_scenario(scenario, seed=args.seed)

    print(f"\nEvacuation time: {result.evacuation_time:.1f}s")
    print(f"Agents evacuated: {result.agents_evacuated} / {result.total_agents}")
    print(f"Agents remaining: {result.agents_remaining}")
    if result.sqlite_file:
        local_path = SCRIPT_DIR / f"{args.model}.sqlite"
        shutil.move(result.sqlite_file, local_path)
        result.sqlite_file = None  # prevent cleanup of moved file
        print(f"Trajectory: {local_path}")


if __name__ == "__main__":
    main()
