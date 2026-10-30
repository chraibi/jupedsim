# SPDX-License-Identifier: LGPL-3.0-or-later
"""Demo scenario: a person standing in a corridor is overtaken.

A 12 m x 3 m corridor leads to an exit at ``x = 12``. One person stands
still in the middle of it and is not controlled by the layer. Four
agents arrive in single file behind it. They queue, stall behind the
standing person, overtake it on a free side and walk on to the exit.
"""

import dataclasses
import pathlib

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
from .scenario import (
    _DT,
    _LOGS,
    _MEMORY,
    _PERCEPTION,
    _check_memory_window,
    _close,
    _rules_metadata,
    _state_type,
)

CORRIDOR = shapely.box(0.0, 0.0, 12.0, 3.0)
STANDING = (6.0, 1.5)
ARRIVALS = ((1.0, 1.5), (1.8, 1.5), (2.6, 1.5), (3.4, 1.5))
DEMO_ENTRANCE = EntranceParameters(entrance=(11.0, 1.5))

_EXIT = shapely.box(11.5, 0.5, 12.0, 2.5)


def build_demo(
    model,
    *,
    selector: Selector,
    log_path: pathlib.Path,
    entrance: EntranceParameters = DEMO_ENTRANCE,
    control_interval: int = 10,
    decision_period: float = 2.0,
) -> tuple[jps.Simulation, RepertoireController, int]:
    """Build the demo with the arriving agents queueing.

    Args:
        model: ``CollisionFreeSpeedModel`` or ``CollisionFreeSpeedModelV3``.
        selector: Chooses the arriving agents' repertoires.
        log_path: File of the run log; closed by :func:`run_demo`.
        entrance: Parameters of the repertoires.
        control_interval: Iterations per controller step.
        decision_period: Seconds between periodic decisions of an agent.

    Returns:
        The simulation, the controller and the id of the standing person.

    Raises:
        ValueError: If the model is not supported, or if the memory window
            is shorter than the longest span the repertoires or rules
            query plus one control interval.
    """
    state_type = _state_type(model)
    _check_memory_window(selector, entrance, control_interval)
    surface = jps.WalkableSurface()
    region = surface.add_region(polygon=CORRIDOR)
    sim = jps.Simulation(model=model, geometry=surface, dt=_DT)
    steering = sim.add_direct_steering_stage()
    steering_journey = sim.add_journey(jps.JourneyDescription([steering]))
    exit_stage = sim.add_exit_stage([(_EXIT, region)])
    exit_journey = sim.add_journey(jps.JourneyDescription([exit_stage]))

    def add(position):
        return sim.add_agent(
            journey_id=steering_journey,
            stage_id=steering,
            position=position,
            state=state_type(),
            region_id=region,
        )

    standing = add(STANDING)
    sim.agent(standing).final_target = STANDING
    ids = [add(pos) for pos in ARRIVALS]
    log = RunLog(
        log_path,
        {
            "scenario": "demo",
            "selector": type(selector).__name__,
            "model": type(model).__name__,
            "standing_agent": standing,
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
            Queueing.name: Queueing(entrance, CORRIDOR),
            Overtaking.name: Overtaking(entrance, CORRIDOR),
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
        walkable=CORRIDOR,
        control_interval=control_interval,
        decision_period=decision_period,
        capabilities=model_capabilities(state_type()),
    )
    for agent_id in ids:
        controller.add(agent_id, Queueing.name)
    _LOGS[controller] = log
    return sim, controller, standing


def run_demo(
    sim: jps.Simulation,
    controller: RepertoireController,
    max_time: float,
    on_step=None,
) -> None:
    """Step until every arriving agent has left or ``max_time`` passed.

    The standing person stays in the corridor. The run log is closed at
    the end.

    Args:
        sim: The simulation.
        controller: The controller built by :func:`build_demo`.
        max_time: Time limit (s).
        on_step: Called with ``(sim, controller)`` before every step,
            e.g. to record frames.
    """
    try:
        while (
            _arrivals_remain(sim, controller) and sim.elapsed_time() < max_time
        ):
            if on_step is not None:
                on_step(sim, controller)
            controller.step()
    finally:
        _close(_LOGS.pop(controller, None))


def _arrivals_remain(sim: jps.Simulation, controller: RepertoireController):
    return sim.agent_count() > 1 or bool(controller.states)
