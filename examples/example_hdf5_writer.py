#! /usr/bin/env python3

# SPDX-License-Identifier: LGPL-3.0-or-later

"""Example: HDF5 trajectory writer.

Runs a short simulation, writes the trajectory to an HDF5 file in a
layout compatible with the loaders provided by `PedPy
<https://github.com/PedestrianDynamics/PedPy>`_ for the Pedestrian
Dynamics Data Archive (PDA) format, then reads the file back with
PedPy and plots the trajectories on top of the walkable area.

Extra dependencies for this example (beyond JuPedSim itself):

    pip install h5py pedpy matplotlib

The script writes ``example_traj.h5`` and ``example_traj.png`` to the
current working directory.
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
        every_nth_frame=4,
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
    for x, y in [(2, 3), (2, 5), (2, 7), (3, 4), (3, 6), (4, 3), (4, 5), (4, 7)]:
        sim.add_agent(
            jps.CollisionFreeSpeedModelV2AgentParameters(
                position=(x, y), journey_id=journey_id, stage_id=exit_id
            )
        )

    while sim.agent_count() > 0 and sim.iteration_count() < 2000:
        sim.iterate()

    writer.close()
    print(
        f"Wrote {out.resolve()} ({sim.iteration_count()} iterations, "
        f"every_nth_frame=4)"
    )

    # --- Read back with PedPy and plot ---------------------------------------
    try:
        import matplotlib.pyplot as plt
        import pedpy
    except ImportError as e:
        print(
            "Skipping plot step: install 'pedpy' and 'matplotlib' "
            f"to enable it ({e})."
        )
        return

    traj = pedpy.load_trajectory_from_ped_data_archive_hdf5(trajectory_file=out)
    walkable_area = pedpy.load_walkable_area_from_ped_data_archive_hdf5(
        trajectory_file=out
    )
    print(
        f"Loaded {len(traj.data)} rows, {traj.data['id'].nunique()} agents, "
        f"fps={traj.frame_rate}"
    )

    fig, ax = plt.subplots(figsize=(8, 6))
    pedpy.plot_trajectories(traj=traj, walkable_area=walkable_area, axes=ax)
    ax.set_aspect("equal")
    ax.set_title("Hdf5TrajectoryWriter -> pedpy.plot_trajectories")
    png = out.with_suffix(".png")
    fig.savefig(png, dpi=120, bbox_inches="tight")
    print(f"Saved plot to {png.resolve()}")


if __name__ == "__main__":
    main()
