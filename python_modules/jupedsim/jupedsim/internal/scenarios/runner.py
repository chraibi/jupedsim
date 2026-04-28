# SPDX-License-Identifier: LGPL-3.0-or-later
"""Build a :class:`jupedsim.Simulation` from a loaded scenario and run it."""

from __future__ import annotations

import pathlib
import random
import tempfile
from dataclasses import dataclass, field
from typing import Any, Mapping, Optional

import jupedsim as jps
from shapely.geometry.base import BaseGeometry

from .builders import build_agent_parameters, build_model
from .loader import LoadedScenario

_DEFAULT_V0 = 1.34
_DEFAULT_RADIUS = 0.15
_DEFAULT_DISTANCE_TO_AGENTS = 0.4
_DEFAULT_DISTANCE_TO_POLYGON = 0.2


# ---------------------------------------------------------------------------
# init_scenario
# ---------------------------------------------------------------------------


def _add_stage(sim: jps.Simulation, spec: Mapping[str, Any]) -> int:
    stype = spec["type"]
    if stype == "waypoint":
        return sim.add_waypoint_stage(
            tuple(spec["position"]), float(spec["distance"])
        )
    if stype == "exit":
        from shapely import from_wkt

        return sim.add_exit_stage(from_wkt(spec["polygon"]))
    if stype == "waiting_set":
        return sim.add_waiting_set_stage([tuple(p) for p in spec["positions"]])
    if stype == "notifiable_queue":
        return sim.add_queue_stage([tuple(p) for p in spec["positions"]])
    if stype == "direct_steering":
        return sim.add_direct_steering_stage()
    raise ValueError(f"Unsupported stage type '{stype}'")


def _add_journey(
    sim: jps.Simulation,
    spec: Mapping[str, Any],
    stage_ids: Mapping[str, int],
) -> int:
    """Build a sequential journey: each stage transitions to the next."""
    ordered_stages = list(spec["stages"])
    if not ordered_stages:
        raise ValueError(f"Journey '{spec.get('name')}' has no stages")
    desc = jps.JourneyDescription([stage_ids[name] for name in ordered_stages])
    for current, nxt in zip(ordered_stages, ordered_stages[1:]):
        desc.set_transition_for_stage(
            stage_ids[current],
            jps.Transition.create_fixed_transition(stage_ids[nxt]),
        )
    return sim.add_journey(desc)


def _agents_in_group(
    group: Mapping[str, Any],
    distribution_polygon: Optional[BaseGeometry],
    rng: random.Random,
) -> list[tuple[float, float]]:
    if "positions" in group:
        return [tuple(p) for p in group["positions"]]
    dist = group["distribution"]
    polygon = distribution_polygon
    assert polygon is not None
    seed = rng.randrange(0, 2**31 - 1)
    return jps.distributions.distribute_by_number(
        polygon=polygon,
        number_of_agents=int(dist["number"]),
        distance_to_agents=float(
            dist.get("min_distance", _DEFAULT_DISTANCE_TO_AGENTS)
        ),
        distance_to_polygon=float(
            dist.get("distance_to_polygon", _DEFAULT_DISTANCE_TO_POLYGON)
        ),
        seed=seed,
    )


def init_scenario(loaded: LoadedScenario) -> jps.Simulation:
    """Build a :class:`jupedsim.Simulation` from a loaded scenario.

    Adds all stages, builds journeys (sequential transitions), and
    places agents according to each agent group spec. Returns the sim
    ready to be iterated; the caller decides how to drive it (the
    typical path is :func:`run_scenario`).
    """
    config = loaded.config
    rng = random.Random(config.seed)

    model = build_model(config.model_type, config.model_params)
    sim = jps.Simulation(
        model=model,
        geometry=loaded.walkable_area,
        dt=config.dt,
    )

    stage_ids: dict[str, int] = {}
    for name, spec in config.stages.items():
        stage_ids[name] = _add_stage(sim, spec)

    journey_ids: dict[str, int] = {}
    for journey_spec in config.journeys:
        name = journey_spec.get("name", f"journey_{len(journey_ids)}")
        journey_ids[name] = _add_journey(sim, journey_spec, stage_ids)

    for idx, group in enumerate(config.agents):
        positions = _agents_in_group(
            group, loaded.distribution_polygons.get(idx), rng
        )
        v0 = float(group.get("v0", _DEFAULT_V0))
        radius = float(group.get("radius", _DEFAULT_RADIUS))
        journey_id = journey_ids[group["journey"]]
        stage_id = stage_ids[group["stage"]]
        for pos in positions:
            sim.add_agent(
                build_agent_parameters(
                    config.model_type,
                    position=pos,
                    journey_id=journey_id,
                    stage_id=stage_id,
                    v0=v0,
                    radius=radius,
                )
            )

    return sim


# ---------------------------------------------------------------------------
# run_scenario
# ---------------------------------------------------------------------------


@dataclass
class ScenarioResult:
    """Outcome of a :func:`run_scenario` call.

    Attributes
    ----------
    success
        True when all agents left the simulation before ``max_time``.
    iterations
        Number of simulation iterations that were executed.
    elapsed_seconds
        Simulated time in seconds (``iterations * dt``).
    agents_remaining
        Agents that did not finish the simulation.
    trajectory_file
        Path to the SQLite (or HDF5) trajectory file written during the run.
    metrics
        Free-form dict for scenario-specific summary statistics.
    """

    success: bool
    iterations: int
    elapsed_seconds: float
    agents_remaining: int
    trajectory_file: pathlib.Path
    metrics: dict[str, Any] = field(default_factory=dict)


def run_scenario(
    sim: jps.Simulation,
    *,
    max_iterations: Optional[int] = None,
    max_time: Optional[float] = None,
    trajectory_writer: Optional[Any] = None,
    output_file: Optional[pathlib.Path] = None,
) -> ScenarioResult:
    """Run a simulation until done or a stopping condition is hit.

    Parameters
    ----------
    sim
        Built by :func:`init_scenario`.
    max_iterations, max_time
        At least one of the two must be supplied (directly or via the
        scenario's ``simulation.max_time``). The simulation stops when
        either is reached or all agents have exited, whichever happens
        first.
    trajectory_writer
        Inject a writer (e.g. :class:`jupedsim.Hdf5TrajectoryWriter`).
        If ``None``, a :class:`jupedsim.SqliteTrajectoryWriter` is used.
    output_file
        Where to put the default sqlite writer's output. Defaults to a
        temporary file. Ignored when ``trajectory_writer`` is supplied.
    """
    if max_iterations is None and max_time is None:
        raise ValueError(
            "run_scenario needs either 'max_iterations' or 'max_time'"
        )
    dt = sim.delta_time()
    if max_iterations is None:
        max_iterations = int(max_time / dt) + 1  # type: ignore[operator]

    if trajectory_writer is None:
        if output_file is None:
            tmp = tempfile.NamedTemporaryFile(suffix=".sqlite", delete=False)
            output_file = pathlib.Path(tmp.name)
            tmp.close()
        trajectory_writer = jps.SqliteTrajectoryWriter(
            output_file=output_file, every_nth_frame=4
        )
        # The simulation does not own the writer; we attach it manually
        # below after iterating because Simulation already accepts a
        # writer at construction time. The init path is handled by
        # init_scenario; here we drive the sim directly.

    # Trajectory writers normally hook into Simulation.iterate via the
    # constructor argument. Simulation does not currently expose a
    # post-construction setter, so this branch supports the case where
    # the caller wants to provide a writer to a sim that was *not*
    # built with one. We honor the contract by calling the writer
    # methods ourselves.
    trajectory_writer.begin_writing(sim)

    iterations = 0
    while sim.agent_count() > 0 and iterations < max_iterations:
        sim.iterate()
        trajectory_writer.write_iteration_state(sim)
        iterations += 1

    if hasattr(trajectory_writer, "close"):
        trajectory_writer.close()

    return ScenarioResult(
        success=sim.agent_count() == 0,
        iterations=iterations,
        elapsed_seconds=iterations * dt,
        agents_remaining=sim.agent_count(),
        trajectory_file=pathlib.Path(output_file)
        if output_file
        else pathlib.Path(""),
    )
