# SPDX-License-Identifier: LGPL-3.0-or-later
"""Entrance experiment 2C120: a corridor in front of a closed door.

People enter a 7 m wide corridor at the times and positions observed in
the experiment and walk to a door at its far end. The door is a wall,
as in the experiment while it is closed. No agent is handed off before
``door_open_time``, so the first agent waits at the door and the others
queue or overtake behind it. Once the door is open, agents within the
handoff radius of the door front follow the exit journey to the niche
in front of the door and leave the simulation there; the corridor
behind the door is not simulated.
"""

import dataclasses
import pathlib

import jupedsim as jps
import numpy as np
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
    _WRITERS,
    _check_memory_window,
    _close,
    _rules_metadata,
    _state_type,
)

_HALL = [(-8.88, -11.1), (8.3, -11.1), (8.3, 27.95), (-8.88, 27.95)]
_LEFT_WALL = [
    (-7, -11), (-3.57, -3), (-3.57, 19.57), (-1.52, 19.57), (-1.37, 19.71),
    (-0.87, 19.71), (-0.72, 19.57), (-0.42, 19.57), (-0.42, 21.23),
    (-0.72, 21.23), (-0.87, 21.09), (-1.37, 21.09), (-1.52, 21.23),
    (-1.67, 21.23), (-1.67, 21.18), (-1.545, 21.18), (-1.42, 21.065),
    (-1.42, 19.735), (-1.545, 19.62), (-3.62, 19.62), (-3.59, -3),
]  # fmt: skip
_RIGHT_WALL = [
    (7, -11), (3.57, -3), (3.64, 19.64), (1.47, 19.57), (1.32, 19.71),
    (0.82, 19.71), (0.67, 19.57), (0.38, 19.57), (0.38, 21.23),
    (0.67, 21.23), (0.82, 21.09), (1.32, 21.09), (1.47, 21.23),
    (1.62, 21.23), (1.62, 21.18), (1.495, 21.18), (1.37, 21.065),
    (1.37, 19.735), (1.495, 19.62), (3.69, 19.69), (3.62, -3),
]  # fmt: skip
_STRIP = [(-6.8, -10.8), (6.8, -10.8), (6.8, -10.6), (-6.8, -10.6)]
_DOOR_LEAF = [(-0.41, 20.0), (0.37, 20.0), (0.37, 21.0), (-0.41, 21.0)]

# Corridor between the walls and the niche in front of the door, where
# the repertoires place their targets; the simulation also covers the
# hall around it.
_CORRIDOR = shapely.box(-3.57, -3.0, 3.57, 19.57)
_NICHE = shapely.box(-0.42, 19.57, 0.38, 20.0)
_EXIT = shapely.box(-0.3, 19.65, 0.26, 19.95)

FPS = 50
DOOR_OPEN_TIME = 90.5
# Desired speeds follow the median of the people's 90th-percentile speeds
# over the first 20 s, 2.5 m/s, with the default ratio of overtaking to
# queueing speed.
ENTRANCE = EntranceParameters(
    entrance=(-0.02, 19.6),
    queue_speed=2.5,
    overtake_speed=3.5,
    door_open_time=DOOR_OPEN_TIME,
)

# Minimum distance to other agents at which an entering agent is added.
_ENTRY_CLEARANCE = 0.4


@dataclasses.dataclass(frozen=True)
class Entry:
    """First observation of one person.

    Attributes:
        person_id: Id in the experiment data.
        time: Time of the first observation since the first one of the
            experiment (s).
        position: Position at the first observation (m).
    """

    person_id: int
    time: float
    position: tuple[float, float]


def geometry() -> shapely.Polygon:
    """Hall with the corridor walls and the closed door as holes."""
    return shapely.Polygon(_HALL, [_LEFT_WALL, _RIGHT_WALL, _STRIP, _DOOR_LEAF])


def corridor() -> shapely.Polygon:
    """Corridor and door niche, in which repertoire targets lie."""
    return shapely.union(_CORRIDOR, _NICHE)


def load_entries(path: pathlib.Path, fps: int = FPS) -> list[Entry]:
    """Read each person's first observation from a PeTrack file.

    Args:
        path: Text file with columns ``id frame x y ...``; lines starting
            with ``#`` are comments.
        fps: Frame rate of the file.

    Returns:
        One entry per person, ordered by time and then person id.
    """
    data = np.loadtxt(path, comments="#", usecols=(0, 1, 2, 3), ndmin=2)
    data = data[np.lexsort((data[:, 1], data[:, 0]))]
    _, first = np.unique(data[:, 0], return_index=True)
    start = data[:, 1].min()
    entries = [
        Entry(
            person_id=int(row[0]),
            time=float((row[1] - start) / fps),
            position=(float(row[2]), float(row[3])),
        )
        for row in data[first]
    ]
    return sorted(entries, key=lambda e: (e.time, e.person_id))


class Inflow:
    """Adds agents at their entry times, once their position is free."""

    def __init__(
        self,
        entries: list[Entry],
        state_type,
        journey_id: int,
        stage_id: int,
        region_id: int,
    ):
        self._pending = list(entries)
        self._state_type = state_type
        self._journey_id = journey_id
        self._stage_id = stage_id
        self._region_id = region_id
        self.added: dict[int, int] = {}

    @property
    def pending(self) -> int:
        """Number of entries not yet added."""
        return len(self._pending)

    def admit(
        self, sim: jps.Simulation, controller: RepertoireController
    ) -> None:
        """Add every due entry whose position is free.

        Entries whose position is occupied wait for a later call. The
        mapping from person id to agent id is kept in :attr:`added`.
        """
        now = sim.elapsed_time()
        waiting = []
        for entry in self._pending:
            if entry.time > now or not self._is_free(sim, entry):
                waiting.append(entry)
                continue
            self.added[entry.person_id] = self._add(sim, controller, entry)
        self._pending = waiting

    def _is_free(self, sim: jps.Simulation, entry: Entry) -> bool:
        return not sim.agents_in_range(entry.position, _ENTRY_CLEARANCE)

    def _add(
        self,
        sim: jps.Simulation,
        controller: RepertoireController,
        entry: Entry,
    ) -> int:
        agent_id = sim.add_agent(
            journey_id=self._journey_id,
            stage_id=self._stage_id,
            position=entry.position,
            state=self._state_type(),
            region_id=self._region_id,
        )
        controller.add(agent_id, Queueing.name)
        return agent_id


def build_2c120(
    model,
    *,
    entries: list[Entry],
    selector: Selector,
    log_path: pathlib.Path,
    entrance: EntranceParameters = ENTRANCE,
    control_interval: int = 10,
    decision_period: float = 2.0,
    trajectory_path: pathlib.Path | None = None,
    every_nth_frame: int = 10,
) -> tuple[jps.Simulation, RepertoireController, Inflow]:
    """Build the 2C120 scenario with no agent yet in the corridor.

    Args:
        model: ``CollisionFreeSpeedModel`` or ``CollisionFreeSpeedModelV3``.
        entries: Entry times and positions, e.g. from :func:`load_entries`.
        selector: Chooses the agents' repertoires.
        log_path: File of the run log; closed by :func:`run_2c120`.
        entrance: Parameters of the repertoires; ``door_open_time`` is the
            time at which the door opens.
        control_interval: Iterations per controller step.
        decision_period: Seconds between periodic decisions of an agent.
        trajectory_path: HDF5 trajectory file to write, or None for none;
            closed by :func:`run_2c120`.
        every_nth_frame: Iterations between recorded trajectory frames.

    Returns:
        The simulation, the controller and the inflow that adds agents.

    Raises:
        ValueError: If the model is not supported, or if the memory window
            is shorter than the longest span the repertoires or rules
            query plus one control interval.
    """
    state_type = _state_type(model)
    _check_memory_window(selector, entrance, control_interval)
    walkable = corridor()
    surface = jps.WalkableSurface()
    region = surface.add_region(polygon=geometry())
    writer = None
    if trajectory_path is not None:
        writer = jps.Hdf5TrajectoryWriter(
            output_file=trajectory_path, every_nth_frame=every_nth_frame
        )
    sim = jps.Simulation(
        model=model, geometry=surface, dt=_DT, trajectory_writer=writer
    )
    steering = sim.add_direct_steering_stage()
    steering_journey = sim.add_journey(jps.JourneyDescription([steering]))
    exit_stage = sim.add_exit_stage([(_EXIT, region)])
    exit_journey = sim.add_journey(jps.JourneyDescription([exit_stage]))
    log = RunLog(
        log_path,
        {
            "scenario": "2C120",
            "selector": type(selector).__name__,
            "n_agents": len(entries),
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
        capabilities=model_capabilities(state_type()),
    )
    inflow = Inflow(entries, state_type, steering_journey, steering, region)
    _LOGS[controller] = log
    if writer is not None:
        _WRITERS[controller] = writer
    return sim, controller, inflow


def run_2c120(
    sim: jps.Simulation,
    controller: RepertoireController,
    inflow: Inflow,
    max_time: float,
) -> None:
    """Admit agents and step the controller until all left or timeout.

    The run ends once every entry was added and every agent left, or at
    ``max_time``. The run log and the trajectory file, if any, are closed
    at the end.
    """
    try:
        while _active(sim, inflow) and sim.elapsed_time() < max_time:
            inflow.admit(sim, controller)
            controller.step()
    finally:
        _close(_LOGS.pop(controller, None))
        _close(_WRITERS.pop(controller, None))


def _active(sim: jps.Simulation, inflow: Inflow) -> bool:
    return sim.agent_count() > 0 or inflow.pending > 0
