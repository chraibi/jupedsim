# SPDX-License-Identifier: LGPL-3.0-or-later
"""Queueing and overtaking repertoires for an entrance approach area."""

import dataclasses
import math
from collections.abc import Callable, Sequence

import shapely
from jupedsim.repertoires import (
    BehaviouralState,
    Capability,
    Command,
    Observation,
    PerceivedAgent,
)
from jupedsim.repertoires.perception import progress_gained

Point = tuple[float, float]

# Events an overtake ends with; each abandonment names its cause.
ABANDONED_NO_GAP = "overtake_abandoned_no_gap"
ABANDONED_TARGET_LOST = "overtake_abandoned_target_lost"
ABANDONED_STALLED = "overtake_abandoned_stalled"


@dataclasses.dataclass(frozen=True)
class EntranceParameters:
    """Parameters of the entrance repertoires.

    Attributes:
        entrance: Entrance centre ``E``.
        ahead_margin: Progress margin for "ahead" (m).
        spacing: Queue spacing behind the predecessor (m).
        standing_tol: Distance to the queue target below which the agent
            counts as standing (m).
        queue_speed: Desired speed while queueing (m/s).
        lateral_offset: Lateral distance of the passing point (m).
        wall_clearance: Distance kept from the walls by passing points (m).
        entry_free_radius: Radius around a passing point that must be
            free of other agents to start an overtake (m).
        continuation_free_radius: Radius around a passing point that must
            stay free of other agents to continue an overtake (m).
        lateral_reach: Distance at which the lateral point counts as
            reached (m).
        ahead_offset: Distance ahead of the passing target in phase 2 (m).
        overtake_speed: Desired speed while overtaking (m/s).
        complete_margin: Progress margin past the target that completes
            an overtake (m).
        abandon_span: Time span of the stall check (s).
        abandon_progress: Minimum progress over ``abandon_span`` (m).
        handoff_radius: Distance to ``E`` below which control is handed
            off (m).
        door_open_time: Time before which no agent is handed off, e.g.
            while the entrance is closed (s).
    """

    entrance: Point = (20.0, 5.0)
    ahead_margin: float = 0.2
    spacing: float = 0.7
    standing_tol: float = 0.15
    queue_speed: float = 1.0
    lateral_offset: float = 0.8
    wall_clearance: float = 0.25
    entry_free_radius: float = 0.6
    continuation_free_radius: float = 0.4
    lateral_reach: float = 0.3
    ahead_offset: float = 0.8
    overtake_speed: float = 1.4
    complete_margin: float = 0.3
    abandon_span: float = 2.0
    abandon_progress: float = 0.1
    handoff_radius: float = 1.0
    door_open_time: float = 0.0


def progress(params: EntranceParameters) -> Callable[[Point], float]:
    """Return the progress coordinate ``p(x) = -|x - E|``."""

    def _progress(pos: Point) -> float:
        return -math.dist(pos, params.entrance)

    return _progress


def heading(params: EntranceParameters) -> Callable[[Point], Point]:
    """Return the map from a position to the unit vector towards ``E``."""

    def _heading(pos: Point) -> Point:
        return _unit(pos, params.entrance)

    return _heading


def _unit(src: Point, dst: Point) -> Point:
    dist = math.dist(src, dst)
    if dist == 0.0:
        return (1.0, 0.0)
    return ((dst[0] - src[0]) / dist, (dst[1] - src[1]) / dist)


def _lateral_candidates(
    params: EntranceParameters, target_pos: Point
) -> tuple[Point, Point]:
    ux, uy = _unit(target_pos, params.entrance)
    nx, ny = -uy, ux
    d = params.lateral_offset
    return (
        (target_pos[0] + d * nx, target_pos[1] + d * ny),
        (target_pos[0] - d * nx, target_pos[1] - d * ny),
    )


def passing_point(
    params: EntranceParameters,
    walkable: shapely.Polygon,
    own: Point,
    target: PerceivedAgent,
    perceived: Sequence[PerceivedAgent],
    free_radius: float,
) -> Point | None:
    """Return the feasible lateral point closer to ``own``, or None.

    A candidate is feasible if it lies inside ``walkable`` shrunk by the
    wall clearance and no perceived agent other than ``target`` is within
    ``free_radius``. Ties go to the ``+n`` side.
    """
    inner = walkable.buffer(-params.wall_clearance)
    others = [p.position for p in perceived if p.id != target.id]
    feasible = [
        c
        for c in _lateral_candidates(params, target.position)
        if _is_feasible(inner, others, c, free_radius)
    ]
    if not feasible:
        return None
    return min(feasible, key=lambda c: math.dist(own, c))


def _is_feasible(
    inner: shapely.Polygon,
    others: Sequence[Point],
    point: Point,
    free_radius: float,
) -> bool:
    if not inner.covers(shapely.Point(point)):
        return False
    return all(math.dist(point, o) >= free_radius for o in others)


def _acquire_predecessor(
    params: EntranceParameters, obs: Observation
) -> PerceivedAgent | None:
    ahead = [
        p
        for p in obs.perceived
        if p.progress > obs.progress + params.ahead_margin
    ]
    if not ahead:
        return None
    return min(ahead, key=lambda p: (math.dist(obs.position, p.position), p.id))


def _find(obs: Observation, agent_id: int | None) -> PerceivedAgent | None:
    return next((p for p in obs.perceived if p.id == agent_id), None)


def _handoff(
    params: EntranceParameters, obs: Observation, state: BehaviouralState
) -> Command | None:
    if obs.time < params.door_open_time:
        return None
    if math.dist(obs.position, params.entrance) >= params.handoff_radius:
        return None
    return Command(
        target=None,
        desired_speed=None,
        execution=state.execution,
        state=state,
        handoff=True,
    )


class Queueing:
    """Follow the nearest agent ahead, or walk to the entrance."""

    name = "queueing"
    requires = frozenset({Capability.TARGET_CONTROL, Capability.DESIRED_SPEED})

    def __init__(self, params: EntranceParameters, walkable: shapely.Polygon):
        self._params = params
        self._walkable = walkable

    def enter(
        self, obs: Observation, state: BehaviouralState
    ) -> BehaviouralState:
        """Acquire a predecessor and start queueing."""
        entered = dataclasses.replace(
            state,
            repertoire=self.name,
            phase=0,
            commitment_since=obs.time,
        )
        return self._follow(
            obs, entered, _acquire_predecessor(self._params, obs)
        )

    def control(self, obs: Observation, state: BehaviouralState) -> Command:
        """Steer to the queue position behind the predecessor."""
        handoff = _handoff(self._params, obs, state)
        if handoff is not None:
            return handoff
        pred = _find(obs, state.anchor)
        if self._needs_reacquisition(obs, state, pred):
            pred = _acquire_predecessor(self._params, obs)
        new_state = self._follow(obs, state, pred)
        return Command(
            target=new_state.target,
            desired_speed=self._params.queue_speed,
            execution=new_state.execution,
            state=new_state,
        )

    def _needs_reacquisition(
        self,
        obs: Observation,
        state: BehaviouralState,
        pred: PerceivedAgent | None,
    ) -> bool:
        if state.anchor is None or obs.memory.anchor_lost or pred is None:
            return True
        return pred.progress <= obs.progress + self._params.ahead_margin

    def _follow(
        self,
        obs: Observation,
        state: BehaviouralState,
        pred: PerceivedAgent | None,
    ) -> BehaviouralState:
        target = self._queue_target(pred)
        standing = math.dist(obs.position, target) < self._params.standing_tol
        return dataclasses.replace(
            state,
            execution="standing" if standing else "advancing",
            anchor=None if pred is None else pred.id,
            target=target,
        )

    def _queue_target(self, pred: PerceivedAgent | None) -> Point:
        if pred is None:
            return self._params.entrance
        ux, uy = _unit(pred.position, self._params.entrance)
        s = self._params.spacing
        return (pred.position[0] - s * ux, pred.position[1] - s * uy)


class Overtaking:
    """Pass the predecessor on a lateral point, then rejoin ahead of it.

    The agent keeps track of the agent it passes within the perception
    range, also once that agent is outside its view cone.
    """

    name = "overtaking"
    requires = frozenset({Capability.TARGET_CONTROL, Capability.DESIRED_SPEED})
    tracks_anchor = True

    def __init__(self, params: EntranceParameters, walkable: shapely.Polygon):
        self._params = params
        self._walkable = walkable

    def enter(
        self, obs: Observation, state: BehaviouralState
    ) -> BehaviouralState:
        """Take the current anchor as the passing target."""
        return dataclasses.replace(
            state,
            repertoire=self.name,
            execution="passing",
            phase=1,
            commitment_since=obs.time,
        )

    def control(self, obs: Observation, state: BehaviouralState) -> Command:
        """Steer along the passing path or finish the overtake."""
        handoff = _handoff(self._params, obs, state)
        if handoff is not None:
            return handoff
        target = _find(obs, state.anchor)
        if target is None:
            return self._abandon(obs, state, ABANDONED_TARGET_LOST)
        if obs.progress > target.progress + self._params.complete_margin:
            return self._complete(state)
        if self._stalled(obs, state):
            return self._abandon(obs, state, ABANDONED_STALLED)
        if state.phase == 1:
            return self._lateral(obs, state, target)
        return self._pass(state, target)

    def _stalled(self, obs: Observation, state: BehaviouralState) -> bool:
        span = self._params.abandon_span
        if obs.time - state.commitment_since < span:
            return False
        gained = progress_gained(obs.memory, obs.time, span)
        return gained < self._params.abandon_progress

    def _lateral(
        self,
        obs: Observation,
        state: BehaviouralState,
        target: PerceivedAgent,
    ) -> Command:
        point = passing_point(
            self._params,
            self._walkable,
            obs.position,
            target,
            obs.perceived,
            self._free_radius(obs, state),
        )
        if point is None:
            return self._abandon(obs, state, ABANDONED_NO_GAP)
        if math.dist(obs.position, point) < self._params.lateral_reach:
            return self._pass(dataclasses.replace(state, phase=2), target)
        return self._command(dataclasses.replace(state, target=point))

    def _free_radius(self, obs: Observation, state: BehaviouralState) -> float:
        # The first control step shares the snapshot of the entry decision,
        # so it chooses the point that satisfied the entry clearance.
        if obs.time == state.commitment_since:
            return self._params.entry_free_radius
        return self._params.continuation_free_radius

    def _pass(self, state: BehaviouralState, target: PerceivedAgent) -> Command:
        ux, uy = _unit(target.position, self._params.entrance)
        d = self._params.ahead_offset
        point = (target.position[0] + d * ux, target.position[1] + d * uy)
        return self._command(dataclasses.replace(state, target=point))

    def _command(self, state: BehaviouralState) -> Command:
        return Command(
            target=state.target,
            desired_speed=self._params.overtake_speed,
            execution="passing",
            state=dataclasses.replace(state, execution="passing"),
        )

    def _complete(self, state: BehaviouralState) -> Command:
        entrance = self._params.entrance
        return Command(
            target=entrance,
            desired_speed=self._params.overtake_speed,
            execution="advancing",
            state=dataclasses.replace(
                state, execution="advancing", target=entrance
            ),
            event="overtake_completed",
        )

    def _abandon(
        self, obs: Observation, state: BehaviouralState, reason: str
    ) -> Command:
        return Command(
            target=obs.position,
            desired_speed=self._params.queue_speed,
            execution="standing",
            state=dataclasses.replace(
                state, execution="standing", target=obs.position
            ),
            event=reason,
        )
