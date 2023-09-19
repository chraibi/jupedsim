#! /usr/bin/env python3

# Copyright © 2012-2023 Forschungszentrum Jülich GmbH
# SPDX-License-Identifier: LGPL-3.0-or-later
import logging
import pathlib
import sys

from shapely import GeometryCollection, Polygon, to_wkt

import jupedsim as jps


def log_debug(msg):
    logging.debug(msg)


def log_info(msg):
    logging.info(msg)


def log_warn(msg):
    logging.warning(msg)


def log_error(msg):
    logging.error(msg)


def main():
    logging.basicConfig(
        level=logging.DEBUG, format="%(levelname)s : %(message)s"
    )
    jps.set_debug_callback(log_debug)
    jps.set_info_callback(log_info)
    jps.set_warning_callback(log_warn)
    jps.set_error_callback(log_error)

    area = GeometryCollection(
        Polygon([(0, 0), (100, 0), (100, 100), (0, 100), (0, 0)])
    )
    geometry = jps.geometry_from_shapely(area)

    model_builder = jps.VelocityModelBuilder(
        a_ped=8, d_ped=0.1, a_wall=5, d_wall=0.02
    )
    model = model_builder.build()

    simulation = jps.Simulation(model=model, geometry=geometry, dt=0.01)
    stage_id = simulation.add_queue_stage(
        [
            (60, 50),
            (59, 50),
            (58, 50),
            (57, 50),
            (56, 50),
            (55, 50),
            (54, 50),
        ]
    )
    queue = simulation.get_stage_proxy(stage_id)
    exit = simulation.add_exit_stage(
        [(99, 40), (99, 60), (100, 60), (100, 40)]
    )

    journey = jps.JourneyDescription([stage_id, exit])
    journey.set_transition_for_stage(
        stage_id, jps.Transition.create_fixed_transition(exit)
    )

    journey_id = simulation.add_journey(journey)

    agent_parameters = jps.VelocityModelAgentParameters()
    agent_parameters.journey_id = journey_id
    agent_parameters.stage_id = stage_id

    agent_parameters.orientation = (1.0, 0.0)
    agent_parameters.position = (0.0, 0.0)
    agent_parameters.time_gap = 1
    agent_parameters.tau = 0.5
    agent_parameters.v0 = 1.2
    agent_parameters.radius = 0.3

    for y in range(1, 16):
        agent_parameters.position = (0.5, y)
        simulation.add_agent(agent_parameters)

    writer = jps.SqliteTrajectoryWriter(pathlib.Path("example3_out.sqlite"))
    writer.begin_writing(25, to_wkt(area, rounding_precision=-1))
    while (
        simulation.agent_count() > 0 and simulation.iteration_count() < 20_000
    ):
        try:
            if (
                simulation.iteration_count() > 100 * 52
                and simulation.iteration_count() % 400 == 0
            ):
                queue.pop(1)
                print("Next!")

            if simulation.iteration_count() % 4 == 0:
                writer.write_iteration_state(simulation)
            simulation.iterate()
        except KeyboardInterrupt:
            writer.end_writing()
            print("CTRL-C Recieved! Shuting down")
            sys.exit(1)

    writer.end_writing()


if __name__ == "__main__":
    main()
