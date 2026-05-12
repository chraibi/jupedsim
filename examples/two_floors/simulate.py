#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-3.0-or-later
"""Two-floor sim that exercises the multi-level plumbing.

Floor 1 (10x10) and floor 2 (10x10) share the same (x,y) footprint. A
2x5 stair geometry connects them at a bottom landing (y in [5,6]) and
a top landing (y in [9,10]). An agent spawned at (1,1) on floor 1 walks
to the bottom landing centroid, gets transferred onto the stair, walks
up to the top landing centroid, gets transferred onto floor 2, and
exits at the bottom-left.

The stages live on the primary geometry for journey-validation purposes;
the level-switch system handles the per-tick crossing.
"""

import pathlib

import jupedsim as jps

FLOOR1 = [(0, 0), (10, 0), (10, 10), (0, 10)]
FLOOR2 = [(0, 0), (10, 0), (10, 10), (0, 10)]
STAIR = [(4, 5), (6, 5), (6, 10), (4, 10)]
BOTTOM_LANDING = [(4, 5), (6, 5), (6, 6), (4, 6)]
TOP_LANDING = [(4, 9), (6, 9), (6, 10), (4, 10)]
EXIT_POLY = [(0, 0), (2, 0), (2, 2), (0, 2)]


def main():
    jps.set_warning_callback(print)
    jps.set_error_callback(print)

    out = pathlib.Path(__file__).parent / "two_floors.sqlite"
    if out.exists():
        out.unlink()

    sim = jps.Simulation(
        model=jps.CollisionFreeSpeedModel(),
        geometry=FLOOR1,
        trajectory_writer=jps.SqliteTrajectoryWriter(output_file=out),
    )

    floor1_id = sim.primary_level
    floor2_id = sim.add_level(FLOOR2)
    stair_id = sim.add_level(STAIR)

    sim.add_landing(
        from_level=floor1_id,
        polygon_from=BOTTOM_LANDING,
        to_level=stair_id,
        polygon_to=BOTTOM_LANDING,
    )
    sim.add_landing(
        from_level=stair_id,
        polygon_from=TOP_LANDING,
        to_level=floor2_id,
        polygon_to=TOP_LANDING,
    )

    bottom_wp = sim.add_waypoint_stage((5.0, 5.5), 0.5)
    top_wp = sim.add_waypoint_stage((5.0, 9.5), 0.5)
    exit_id = sim.add_exit_stage(EXIT_POLY)

    journey = jps.JourneyDescription([bottom_wp, top_wp, exit_id])
    journey.set_transition_for_stage(
        bottom_wp, jps.Transition.create_fixed_transition(top_wp)
    )
    journey.set_transition_for_stage(
        top_wp, jps.Transition.create_fixed_transition(exit_id)
    )
    journey_id = sim.add_journey(journey)

    aid = sim.add_agent(
        jps.CollisionFreeSpeedModelAgentParameters(
            journey_id=journey_id, stage_id=bottom_wp, position=(1.0, 1.0), radius=0.2
        )
    )

    print(
        f"Levels: floor1={floor1_id} floor2={floor2_id} stair={stair_id}"
    )
    print("Running...")

    last_level = None
    for _ in range(5000):
        sim.iterate()
        if sim.agent_count() == 0:
            break
        a = sim.agent(aid)
        if a.level != last_level:
            print(
                f"  iter={sim.iteration_count():5d}  pos=({a.position[0]:.2f}, {a.position[1]:.2f})  level={a.level}"
            )
            last_level = a.level

    print(
        f"Finished after {sim.iteration_count()} iterations, "
        f"{sim.agent_count()} agent(s) remaining"
    )


if __name__ == "__main__":
    main()
