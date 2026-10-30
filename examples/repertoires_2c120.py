#! /usr/bin/env python3

# SPDX-License-Identifier: LGPL-3.0-or-later
"""Run the 2C120 entrance experiment with repertoires and summarise it."""

import argparse
import collections
import dataclasses
import pathlib

import jupedsim as jps
from jupedsim.repertoires import FixedSelector, ReplaySelector
from jupedsim.repertoires.log import load_records
from jupedsim_examples.repertoires import EntranceRuleSelector, RuleParameters
from jupedsim_examples.repertoires.croma import (
    DOOR_OPEN_TIME,
    ENTRANCE,
    build_2c120,
    corridor,
    load_entries,
    run_2c120,
)

MODELS = {
    "cfsm": jps.CollisionFreeSpeedModel,
    "cfsm-v3": jps.CollisionFreeSpeedModelV3,
    "warpdriver": jps.WarpDriverModel,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "data", type=pathlib.Path, help="PeTrack trajectory file of 2C120"
    )
    parser.add_argument("--model", choices=sorted(MODELS), default="cfsm-v3")
    parser.add_argument(
        "--selector", choices=["rules", "fixed", "replay"], default="rules"
    )
    parser.add_argument(
        "--log", type=pathlib.Path, default=pathlib.Path("2c120.jsonl")
    )
    parser.add_argument("--replay-from", type=pathlib.Path)
    parser.add_argument("--door-open-time", type=float, default=DOOR_OPEN_TIME)
    parser.add_argument("--max-time", type=float, default=200.0)
    parser.add_argument(
        "--trajectory",
        type=pathlib.Path,
        help="HDF5 trajectory file to write (default: none)",
    )
    parser.add_argument("--every-nth-frame", type=int, default=10)
    args = parser.parse_args()
    if args.selector == "replay" and args.replay_from is None:
        parser.error("--selector replay requires --replay-from")
    return args


def make_selector(args: argparse.Namespace, entrance):
    if args.selector == "fixed":
        return FixedSelector()
    if args.selector == "replay":
        return ReplaySelector(args.replay_from)
    return EntranceRuleSelector(RuleParameters(), entrance, corridor())


def print_summary(log: pathlib.Path) -> None:
    records = load_records(log)
    transitions = collections.Counter(
        r["after"] for r in records if r["type"] == "transition"
    )
    reasons = collections.Counter(
        reason
        for r in records
        if r["type"] == "response"
        for reason in r.get("reasons", ())
    )
    handoffs = sum(r["type"] == "handoff" for r in records)
    print(f"transitions: {dict(transitions)}")
    print(f"decision reasons: {dict(reasons)}")
    print(f"handoffs: {handoffs}")


def main() -> None:
    args = parse_args()
    entrance = dataclasses.replace(ENTRANCE, door_open_time=args.door_open_time)
    sim, controller, inflow = build_2c120(
        MODELS[args.model](),
        entries=load_entries(args.data),
        selector=make_selector(args, entrance),
        log_path=args.log,
        entrance=entrance,
        trajectory_path=args.trajectory,
        every_nth_frame=args.every_nth_frame,
    )
    run_2c120(sim, controller, inflow, args.max_time)
    print(
        f"t={sim.elapsed_time():.2f} s, "
        f"{sim.agent_count()} agents remaining, "
        f"{inflow.pending} not entered, log: {args.log}"
        + (f", trajectory: {args.trajectory}" if args.trajectory else "")
    )
    print_summary(args.log)


if __name__ == "__main__":
    main()
