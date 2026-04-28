#! /usr/bin/env python3

# SPDX-License-Identifier: LGPL-3.0-or-later

"""Example: HDF5 trajectory writer.

Writes a short simulation to an HDF5 file in a layout compatible with the
loaders provided by `PedPy
<https://github.com/PedestrianDynamics/PedPy>`_ for the Pedestrian
Dynamics Data Archive (PDA) format. After running, you can load the
resulting file with::

    import h5py
    import pandas as pd

    with h5py.File("example_traj.h5", "r") as hf:
        df = pd.DataFrame(hf["trajectory"][:])
        fps = hf["trajectory"].attrs["fps"]
        wkt = hf.attrs["wkt_geometry"]

or directly with PedPy::

    from pedpy.io import load_trajectory_from_ped_data_archive_hdf5
    traj = load_trajectory_from_ped_data_archive_hdf5(
        trajectory_file="example_traj.h5"
    )
"""

import pathlib
import sys

import jupedsim as jps
from shapely import GeometryCollection, Polygon


def main() -> None:
    if jps.Hdf5TrajectoryWriter is None:
        sys.exit("h5py is not installed. Install with: pip install h5py")

    area = GeometryCollection(Polygon([(0, 0), (10, 0), (10, 10), (0, 10)]))
    out = pathlib.Path("example_traj.h5")
    writer = jps.Hdf5TrajectoryWriter(
        output_file=out,
        every_nth_frame=1,
        compression_level=4,
    )

    sim = jps.Simulation(
        model=jps.CollisionFreeSpeedModelV2(),
        geometry=area,
        trajectory_writer=writer,
        dt=0.01,
    )
    exit_id = sim.add_exit_stage(Polygon([(9, 0), (10, 0), (10, 10), (9, 10)]))
    journey_id = sim.add_journey(jps.JourneyDescription([exit_id]))
    for x, y in [(2, 5), (3, 4), (3, 6)]:
        sim.add_agent(
            jps.CollisionFreeSpeedModelV2AgentParameters(
                position=(x, y), journey_id=journey_id, stage_id=exit_id
            )
        )

    while sim.agent_count() > 0 and sim.iteration_count() < 500:
        sim.iterate()

    writer.close()
    print(f"Wrote {out.resolve()}")


if __name__ == "__main__":
    main()
