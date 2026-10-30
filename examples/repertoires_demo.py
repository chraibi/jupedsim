#! /usr/bin/env python3

# SPDX-License-Identifier: LGPL-3.0-or-later
"""Run the repertoire demo and animate it, coloured by repertoire."""

import argparse
import pathlib

import jupedsim as jps
import matplotlib.pyplot as plt
from jupedsim.repertoires.log import transitions_by_agent
from jupedsim_examples.repertoires import EntranceRuleSelector, RuleParameters
from jupedsim_examples.repertoires.demo import (
    CORRIDOR,
    DEMO_ENTRANCE,
    build_demo,
    run_demo,
)
from matplotlib.animation import FuncAnimation, PillowWriter

MODELS = {
    "cfsm": jps.CollisionFreeSpeedModel,
    "cfsm-v3": jps.CollisionFreeSpeedModelV3,
}
COLOURS = {
    "queueing": "#4c72b0",
    "overtaking": "#dd8452",
    "exit": "#8c8c8c",
    "standing": "#222222",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", choices=sorted(MODELS), default="cfsm")
    parser.add_argument(
        "--log", type=pathlib.Path, default=pathlib.Path("demo.jsonl")
    )
    parser.add_argument(
        "--gif", type=pathlib.Path, default=pathlib.Path("demo.gif")
    )
    parser.add_argument("--max-time", type=float, default=60.0)
    return parser.parse_args()


def recorder(frames: list, standing: int):
    def record(sim, controller):
        labels = []
        for agent in sim.agents():
            state = controller.states.get(agent.id)
            if agent.id == standing:
                label = "standing"
            elif state is None:
                label = "exit"
            else:
                label = state.repertoire
            labels.append((*agent.position, label))
        frames.append((sim.elapsed_time(), labels))

    return record


def animate(frames: list, path: pathlib.Path) -> None:
    fig, ax = plt.subplots(figsize=(8, 2.8))
    ax.plot(*CORRIDOR.exterior.xy, color="black", linewidth=1)
    ax.plot(*DEMO_ENTRANCE.entrance, marker="x", color="black")
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    for label, colour in COLOURS.items():
        ax.scatter([], [], color=colour, label=label)
    ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), frameon=False)
    dots = ax.scatter([], [], s=60)
    fig.tight_layout()

    def draw(k):
        time, labels = frames[k]
        dots.set_offsets([(x, y) for x, y, _ in labels])
        dots.set_color([COLOURS[label] for _, _, label in labels])
        ax.set_title(f"t = {time:.1f} s")
        return (dots,)

    anim = FuncAnimation(fig, draw, frames=len(frames), blit=False)
    anim.save(path, writer=PillowWriter(fps=10))
    plt.close(fig)


def main() -> None:
    args = parse_args()
    sim, controller, standing = build_demo(
        MODELS[args.model](),
        selector=EntranceRuleSelector(
            RuleParameters(), DEMO_ENTRANCE, CORRIDOR
        ),
        log_path=args.log,
    )
    frames = []
    run_demo(sim, controller, args.max_time, recorder(frames, standing))
    animate(frames, args.gif)
    print(f"t={sim.elapsed_time():.1f} s, log: {args.log}, gif: {args.gif}")
    for agent_id, records in sorted(transitions_by_agent(args.log).items()):
        steps = ", ".join(
            f"t={r['time']:.1f} {r['before']}→{r['after']} ({r['cause']})"
            for r in records
        )
        print(f"agent {agent_id}: {steps}")


if __name__ == "__main__":
    main()
