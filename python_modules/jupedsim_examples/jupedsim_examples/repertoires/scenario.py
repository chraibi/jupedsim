# SPDX-License-Identifier: LGPL-3.0-or-later
"""Entrance scenario: an approach area in front of a narrow corridor.

Agents start in a 20 m x 10 m approach area and walk to an entrance at
``x = 20``, behind which a 2 m x 1 m corridor leads to the exit. They
queue or overtake under a :class:`RepertoireController` until they reach
the entrance, then follow the exit journey.
"""

import dataclasses
import pathlib
import weakref

import jupedsim as jps
import shapely
from jupedsim.repertoires import (
    RepertoireController,
    Selector,
    model_capabilities,
)
from jupedsim.repertoires.log import RunLog
from jupedsim.repertoires.perception import MemoryTracker, Perception

from .entrance import (
    EntranceParameters,
    Overtaking,
    Queueing,
    heading,
    progress,
)
from .rules import EntranceRuleSelector

_APPROACH = shapely.box(0, 0, 20, 10)
_CORRIDOR = shapely.box(20, 4.5, 22, 5.5)
_EXIT = shapely.box(21.5, 4.5, 22, 5.5)
_START = shapely.box(2, 0, 12, 10)

_PERCEPTION = {"radius": 4.0, "half_angle_deg": 90.0}
_MEMORY = {"window": 8.0, "pass_tolerance": 0.2}
_DT = 0.01

_STATE_TYPES = {
    jps.CollisionFreeSpeedModel: jps.CollisionFreeSpeedModelState,
    jps.CollisionFreeSpeedModelV3: jps.CollisionFreeSpeedModelV3State,
}

# Run log of each controller built here, closed by ``run``.
_LOGS: "weakref.WeakKeyDictionary[RepertoireController, RunLog]" = (
    weakref.WeakKeyDictionary()
)


def walkable_area() -> shapely.Polygon:
    """Approach area and entrance corridor as one polygon."""
    return shapely.union(_APPROACH, _CORRIDOR)


def _state_type(model):
    state_type = _STATE_TYPES.get(type(model))
    if state_type is None:
        raise ValueError(f"unsupported model {type(model).__name__}")
    return state_type


def _rules_metadata(selector: Selector) -> dict:
    if not isinstance(selector, EntranceRuleSelector):
        return {}
    return {"rules": dataclasses.asdict(selector.rules)}


def _check_memory_window(
    selector: Selector, entrance: EntranceParameters, control_interval: int
) -> None:
    spans = [entrance.abandon_span]
    if isinstance(selector, EntranceRuleSelector):
        spans += [selector.rules.t_stall, selector.rules.t_passes]
    needed = max(spans) + control_interval * _DT
    if _MEMORY["window"] < needed:
        raise ValueError(
            f"memory window {_MEMORY['window']} s is shorter than the "
            f"longest queried span plus one control interval, {needed} s"
        )


def build_entrance(
    model,
    *,
    n_agents: int,
    seed: int,
    selector: Selector,
    log_path: pathlib.Path,
    entrance: EntranceParameters = EntranceParameters(),
    control_interval: int = 10,
    decision_period: float = 2.0,
) -> tuple[jps.Simulation, RepertoireController]:
    """Build the entrance scenario with all agents queueing.

    Args:
        model: ``CollisionFreeSpeedModel`` or ``CollisionFreeSpeedModelV3``.
        n_agents: Number of agents placed in ``x`` in ``[2, 12]``.
        seed: Seed of the agent placement.
        selector: Chooses the agents' repertoires.
        log_path: File of the run log; closed by :func:`run`.
        entrance: Parameters of the repertoires.
        control_interval: Iterations per controller step.
        decision_period: Seconds between periodic decisions of an agent.

    Returns:
        The simulation and the controller driving its agents.

    Raises:
        ValueError: If the model is not supported, or if the memory window
            is shorter than the longest span the repertoires or rules
            query plus one control interval.
    """
    state_type = _state_type(model)
    _check_memory_window(selector, entrance, control_interval)
    walkable = walkable_area()
    surface = jps.WalkableSurface()
    region = surface.add_region(polygon=walkable)
    sim = jps.Simulation(model=model, geometry=surface, dt=_DT)
    steering = sim.add_direct_steering_stage()
    steering_journey = sim.add_journey(jps.JourneyDescription([steering]))
    exit_stage = sim.add_exit_stage([(_EXIT, region)])
    exit_journey = sim.add_journey(jps.JourneyDescription([exit_stage]))
    positions = jps.distribute_by_number(
        polygon=_START,
        number_of_agents=n_agents,
        distance_to_agents=0.5,
        distance_to_polygon=0.3,
        seed=seed,
    )
    ids = [
        sim.add_agent(
            journey_id=steering_journey,
            stage_id=steering,
            position=pos,
            state=state_type(),
            region_id=region,
        )
        for pos in positions
    ]
    log = RunLog(
        log_path,
        {
            "selector": type(selector).__name__,
            "seed": seed,
            "n_agents": n_agents,
            "model": type(model).__name__,
            "entrance": dataclasses.asdict(entrance),
            "control_interval": control_interval,
            "decision_period": decision_period,
            "jupedsim_version": jps.__version__,
            "perception": _PERCEPTION,
            "memory": _MEMORY,
            **_rules_metadata(selector),
        },
    )
    controller = RepertoireController(
        sim,
        repertoires={
            Queueing.name: Queueing(entrance, walkable),
            Overtaking.name: Overtaking(entrance, walkable),
        },
        selector=selector,
        perception=Perception(
            progress=progress(entrance),
            heading=heading(entrance),
            **_PERCEPTION,
        ),
        memory=MemoryTracker(**_MEMORY),
        log=log,
        steering_journey=steering_journey,
        steering_stage=steering,
        exit_journey=exit_journey,
        exit_stage=exit_stage,
        walkable=walkable,
        control_interval=control_interval,
        decision_period=decision_period,
        capabilities=model_capabilities(sim.agent(ids[0]).state),
    )
    for agent_id in ids:
        controller.add(agent_id, Queueing.name)
    _LOGS[controller] = log
    return sim, controller


def run(
    sim: jps.Simulation, controller: RepertoireController, max_time: float
) -> None:
    """Step the controller until all agents left or ``max_time`` passed.

    The run log is closed at the end.
    """
    try:
        while sim.agent_count() > 0 and sim.elapsed_time() < max_time:
            controller.step()
    finally:
        log = _LOGS.pop(controller, None)
        if log is not None:
            log.close()
