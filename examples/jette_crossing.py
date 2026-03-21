# SPDX-License-Identifier: LGPL-3.0-or-later
"""Jette Crossing scenario: compare WarpDriver, CollisionFreeSpeedModel,
and SocialForceModel on the same geometry and agent setup.

Reads geometry and config from ~/Downloads/jps_2026_03_21_13_23_19/.
Agents are spawned at 10/s per distribution area during the simulation loop.

Journey structure (from config.json):
  Journey 0: dist_0 -> cp_0 -> exit_0 (5%) / exit_1 (15%) / exit_2 (80%)
  Journey 1: dist_1 -> cp_1 -> exit_1 (67%) / exit_4 (33%)
  Journey 2: dist_2 -> cp_2 -> exit_2 (25%) / exit_3 (75%)
"""

from __future__ import annotations

import json
import time
from pathlib import Path

import jupedsim as jps
import matplotlib.pyplot as plt
import shapely
from shapely import wkt

CONFIG_DIR = Path.home() / "Downloads" / "jps_2026_03_21_13_30_43"

MODELS = {
    "WarpDriver": (
        jps.WarpDriverModel,
        jps.WarpDriverModelAgentParameters,
    ),
    "CFS": (
        jps.CollisionFreeSpeedModel,
        jps.CollisionFreeSpeedModelAgentParameters,
    ),
    "SocialForce": (
        jps.SocialForceModel,
        jps.SocialForceModelAgentParameters,
    ),
}

_HAS_ORIENTATION = {
    jps.SocialForceModelAgentParameters,
    jps.WarpDriverModelAgentParameters,
}

SPAWN_RATE = 10  # agents per second per distribution
DT = 0.01


def load_config():
    with open(CONFIG_DIR / "config.json") as f:
        return json.load(f)


def load_geometry():
    with open(CONFIG_DIR / "geometry.wkt") as f:
        return wkt.loads(f.read())


def polygon_from_coords(coords):
    return shapely.Polygon(coords)


def setup_and_run(model_name, seed=420):
    cfg = load_config()
    geometry = load_geometry()
    max_time = cfg["config"]["simulation_settings"]["simulationParams"]["max_simulation_time"]
    max_steps = int(max_time / DT)

    model_cls, agent_cls = MODELS[model_name]
    model = model_cls()

    sqlite_file = f"jette_{model_name.lower()}.sqlite"
    writer = jps.SqliteTrajectoryWriter(output_file=sqlite_file)
    sim = jps.Simulation(model=model, geometry=geometry, dt=DT, trajectory_writer=writer)

    # ── Create all exit stages ──
    exit_ids = {}
    for exit_name, exit_data in cfg["exits"].items():
        eid = sim.add_exit_stage(polygon_from_coords(exit_data["coordinates"]))
        exit_ids[exit_name] = eid

    # ── Create all checkpoint stages (as waypoints at centroid) ──
    cp_ids = {}
    for cp_name, cp_data in cfg["checkpoints"].items():
        cp_poly = polygon_from_coords(cp_data["coordinates"])
        centroid = cp_poly.centroid
        # Use half the shorter dimension as waypoint radius so agents reliably trigger it
        bounds = cp_poly.bounds
        half_w = (bounds[2] - bounds[0]) / 2
        half_h = (bounds[3] - bounds[1]) / 2
        wp_radius = min(half_w, half_h)
        wid = sim.add_waypoint_stage((centroid.x, centroid.y), wp_radius)
        cp_ids[cp_name] = wid

    # ── Build journeys with round-robin routing at checkpoints ──
    routing = cfg["waypoint_routing"]
    journey_ids = {}  # journey config id -> (jps journey id, first waypoint id)

    for journey_cfg in cfg["journeys"]:
        jcfg_id = journey_cfg["id"]
        stages = journey_cfg["stages"]
        transitions = journey_cfg["transitions"]

        # Collect all stage ids used in this journey
        all_stage_ids = set()
        for stage_name in stages:
            if stage_name in exit_ids:
                all_stage_ids.add(exit_ids[stage_name])
            elif stage_name in cp_ids:
                all_stage_ids.add(cp_ids[stage_name])
            # distributions are not simulation stages

        journey = jps.JourneyDescription(list(all_stage_ids))

        # Find the first checkpoint (entry point for agents from distribution)
        first_cp_name = None
        for t in transitions:
            if t["from"].startswith("jps-distributions"):
                first_cp_name = t["to"]
                break

        # Set routing at checkpoint using round-robin weights from waypoint_routing
        for cp_name, cp_routing in routing.items():
            if jcfg_id not in cp_routing:
                continue
            cp_wid = cp_ids[cp_name]
            destinations = cp_routing[jcfg_id]["destinations"]
            weights = [(exit_ids[d["target"]], d["percentage"]) for d in destinations]
            journey.set_transition_for_stage(
                cp_wid,
                jps.Transition.create_round_robin_transition(weights),
            )

        # Set fixed transition from first checkpoint if there's a chain
        # (checkpoint -> checkpoint), but in this config each journey has
        # distribution -> checkpoint -> exits, so the checkpoint routing above
        # handles everything.

        jid = sim.add_journey(journey)
        journey_ids[jcfg_id] = (jid, cp_ids[first_cp_name])

    # ── Prepare spawn queues for each distribution ──
    # Map each distribution to its journey
    dist_journey_map = {}
    for journey_cfg in cfg["journeys"]:
        jcfg_id = journey_cfg["id"]
        for t in journey_cfg["transitions"]:
            if t["from"].startswith("jps-distributions"):
                dist_journey_map[t["from"]] = jcfg_id

    spawn_configs = []
    for dist_name, dist_data in cfg["distributions"].items():
        params = dist_data["parameters"]
        dist_poly = polygon_from_coords(dist_data["coordinates"])
        n_agents = params["number"]
        v0 = params["v0"]
        radius = params["radius"]

        jcfg_id = dist_journey_map[dist_name]
        jid, first_wid = journey_ids[jcfg_id]

        # Pre-generate all spawn positions
        positions = jps.distributions.distribute_by_number(
            polygon=dist_poly,
            number_of_agents=n_agents,
            distance_to_agents=0.4,
            distance_to_polygon=0.15,
            seed=seed,
        )

        spawn_configs.append({
            "positions": list(positions),
            "v0": v0,
            "radius": radius,
            "journey_id": jid,
            "stage_id": first_wid,
            "spawned": 0,
        })

    # ── Simulation loop with flow spawning ──
    steps_per_spawn = int(1.0 / (SPAWN_RATE * DT))  # 10 agents/s -> every 10 steps
    total_spawned = 0

    print(f"[{model_name}] Spawning {SPAWN_RATE}/s per distribution, "
          f"{sum(len(s['positions']) for s in spawn_configs)} agents total...")

    t0 = time.perf_counter()
    step = 0
    for step in range(1, max_steps + 1):
        # Spawn agents at the configured rate
        if step % steps_per_spawn == 0:
            for sc in spawn_configs:
                if sc["spawned"] < len(sc["positions"]):
                    pos = sc["positions"][sc["spawned"]]
                    kwargs = dict(
                        position=pos,
                        journey_id=sc["journey_id"],
                        stage_id=sc["stage_id"],
                        desired_speed=sc["v0"],
                        radius=sc["radius"],
                    )
                    if agent_cls in _HAS_ORIENTATION:
                        kwargs["orientation"] = (1, 0)
                    try:
                        sim.add_agent(agent_cls(**kwargs))
                        sc["spawned"] += 1
                        total_spawned += 1
                    except RuntimeError:
                        pass  # too close to existing agent, retry next tick

        sim.iterate()

        # Stop when all agents spawned and all exited
        all_spawned = all(sc["spawned"] >= len(sc["positions"]) for sc in spawn_configs)
        if all_spawned and sim.agent_count() == 0:
            break

    elapsed = time.perf_counter() - t0
    remaining = sim.agent_count()
    print(
        f"[{model_name}] Done: {step} steps ({step * DT:.1f}s sim), "
        f"{elapsed:.1f}s wall, {total_spawned - remaining}/{total_spawned} exited"
    )
    return sqlite_file, step


def plot_trajectories(sqlite_files):
    import pedpy

    _, axes = plt.subplots(1, len(sqlite_files), figsize=(6 * len(sqlite_files), 6))
    if len(sqlite_files) == 1:
        axes = [axes]

    geometry = load_geometry()

    for ax, (model_name, sqlite_file) in zip(axes, sqlite_files.items()):
        traj = pedpy.load_trajectory_from_jupedsim_sqlite(
            trajectory_file=Path(sqlite_file),
        )
        ax.set_title(model_name)
        pedpy.plot_trajectories(
            traj=traj,
            walkable_area=pedpy.WalkableArea(geometry),
            axes=ax,
        )
        ax.set_aspect("equal")

    plt.tight_layout()
    plt.savefig("jette_crossing_comparison.png", dpi=150)
    print("Saved jette_crossing_comparison.png")
    plt.show()


if __name__ == "__main__":
    results = {}
    for name in MODELS:
        sqlite_file, steps = setup_and_run(name)
        results[name] = sqlite_file

    plot_trajectories(results)
