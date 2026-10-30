#! /usr/bin/env python3

# SPDX-License-Identifier: LGPL-3.0-or-later
"""Run the entrance repertoire example and print each agent's timeline."""

import argparse
import pathlib

import jupedsim as jps
from jupedsim.repertoires import FixedSelector, ReplaySelector
from jupedsim.repertoires.log import transitions_by_agent
from jupedsim_examples.repertoires import (
    EntranceParameters,
    EntranceRuleSelector,
    RuleParameters,
)
from jupedsim_examples.repertoires.scenario import (
    build_entrance,
    run,
    walkable_area,
)

MODELS = {
    "cfsm": jps.CollisionFreeSpeedModel,
    "cfsm-v3": jps.CollisionFreeSpeedModelV3,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", choices=sorted(MODELS), default="cfsm")
    parser.add_argument(
        "--selector", choices=["rules", "fixed", "replay"], default="rules"
    )
    parser.add_argument(
        "--log", type=pathlib.Path, default=pathlib.Path("entrance.jsonl")
    )
    parser.add_argument("--replay-from", type=pathlib.Path)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--agents", type=int, default=30)
    parser.add_argument("--max-time", type=float, default=120.0)
    args = parser.parse_args()
    if args.selector == "replay" and args.replay_from is None:
        parser.error("--selector replay requires --replay-from")
    return args


def make_selector(args: argparse.Namespace):
    if args.selector == "fixed":
        return FixedSelector()
    if args.selector == "replay":
        return ReplaySelector(args.replay_from)
    return EntranceRuleSelector(
        RuleParameters(), EntranceParameters(), walkable_area()
    )


def print_timeline(log: pathlib.Path) -> None:
    for agent_id, records in sorted(transitions_by_agent(log).items()):
        steps = ", ".join(
            f"t={r['time']:.2f} {r['before']}→{r['after']} ({r['cause']})"
            for r in records
        )
        print(f"agent {agent_id}: {steps}")


def main() -> None:
    args = parse_args()
    sim, controller = build_entrance(
        MODELS[args.model](),
        n_agents=args.agents,
        seed=args.seed,
        selector=make_selector(args),
        log_path=args.log,
    )
    run(sim, controller, args.max_time)
    print(
        f"t={sim.elapsed_time():.2f} s, "
        f"{sim.agent_count()} agents remaining, log: {args.log}"
    )
    print_timeline(args.log)


if __name__ == "__main__":
    main()
