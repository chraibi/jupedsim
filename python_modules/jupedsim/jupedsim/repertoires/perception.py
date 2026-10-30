# SPDX-License-Identifier: LGPL-3.0-or-later

import dataclasses
import math
from collections.abc import Callable, Sequence

from .types import Memory, Observation, PerceivedAgent


class Perception:
    """Builds an agent's observation from the simulation.

    Args:
        progress: Maps a position to progress along the route.
        radius: Perception range.
        half_angle_deg: Half opening angle of the view cone in degrees.
        heading: Maps a position to the unit direction of the view cone.
    """

    def __init__(
        self,
        progress: Callable[[tuple[float, float]], float],
        radius: float,
        half_angle_deg: float,
        heading: Callable[[tuple[float, float]], tuple[float, float]],
    ):
        self._progress = progress
        self._radius = radius
        self._cos_half_angle = math.cos(math.radians(half_angle_deg))
        self._heading = heading

    def observe(
        self, sim, agent_id: int, time: float, memory: Memory
    ) -> Observation:
        """Observe the surroundings of one agent.

        Args:
            sim: The simulation.
            agent_id: Id of the observing agent.
            time: Current simulation time.
            memory: Memory of the observing agent.

        Returns:
            The agent's observation. Perceived agents are sorted by id.
        """
        own = tuple(sim.agent(agent_id).position)
        heading = self._heading(own)
        perceived = [
            PerceivedAgent(
                id=other.id,
                position=tuple(other.position),
                progress=self._progress(tuple(other.position)),
            )
            for other in sim.agents_in_range(own, self._radius)
            if other.id != agent_id
            and self._in_cone(own, heading, tuple(other.position))
        ]
        perceived.sort(key=lambda p: p.id)
        return Observation(
            agent_id=agent_id,
            time=time,
            position=own,
            progress=self._progress(own),
            perceived=tuple(perceived),
            memory=memory,
        )

    def _in_cone(
        self,
        own: tuple[float, float],
        heading: tuple[float, float],
        other: tuple[float, float],
    ) -> bool:
        dx, dy = other[0] - own[0], other[1] - own[1]
        dist = math.hypot(dx, dy)
        if dist == 0.0:
            return True
        dot = dx * heading[0] + dy * heading[1]
        return dot >= self._cos_half_angle * dist


@dataclasses.dataclass
class _AgentRecord:
    history: list[tuple[float, float]] = dataclasses.field(default_factory=list)
    passes: list[tuple[float, int]] = dataclasses.field(default_factory=list)
    behind_since: dict[int, float] = dataclasses.field(default_factory=dict)
    anchor_lost: bool = False


class MemoryTracker:
    """Maintains each agent's memory of progress and of being passed.

    A pass by ``j`` is recorded for ``i`` only if ``j`` was perceived on
    consecutive updates, first with ``p(j) < p(i) - tol`` and later with
    ``p(j) > p(i) + tol``. If ``j`` was not perceived on the previous
    update, its history restarts.

    Args:
        window: Time span after which history and passes are dropped.
        pass_tolerance: Progress margin ``tol`` of the pass rule.
    """

    def __init__(self, window: float, pass_tolerance: float):
        self._window = window
        self._tol = pass_tolerance
        self._records: dict[int, _AgentRecord] = {}

    def update(
        self,
        agent_id: int,
        time: float,
        own_progress: float,
        perceived: Sequence[PerceivedAgent],
    ) -> Memory:
        """Record one observation and return the agent's memory."""
        rec = self._records.setdefault(agent_id, _AgentRecord())
        rec.history.append((time, own_progress))
        self._update_passes(rec, time, own_progress, perceived)
        cutoff = time - self._window
        rec.history = [h for h in rec.history if h[0] >= cutoff]
        rec.passes = [p for p in rec.passes if p[0] >= cutoff]
        return Memory(
            progress_history=tuple(rec.history),
            passes=tuple(rec.passes),
            anchor_lost=rec.anchor_lost,
        )

    def forget(self, agent_id: int) -> None:
        """Drop everything remembered by and about an agent."""
        self._records.pop(agent_id, None)
        for rec in self._records.values():
            rec.behind_since.pop(agent_id, None)

    def mark_anchor_lost(self, agent_id: int) -> None:
        """Flag that the agent's anchor has left the simulation."""
        self._records.setdefault(agent_id, _AgentRecord()).anchor_lost = True

    def clear_anchor_lost(self, agent_id: int) -> None:
        """Clear the anchor-lost flag of the agent."""
        rec = self._records.get(agent_id)
        if rec is not None:
            rec.anchor_lost = False

    def _update_passes(
        self,
        rec: _AgentRecord,
        time: float,
        own_progress: float,
        perceived: Sequence[PerceivedAgent],
    ) -> None:
        seen = {p.id for p in perceived}
        rec.behind_since = {
            j: t for j, t in rec.behind_since.items() if j in seen
        }
        cutoff = time - self._window
        for p in perceived:
            rel = p.progress - own_progress
            if rel < -self._tol:
                rec.behind_since[p.id] = time
            elif rel > self._tol and self._was_behind(rec, p.id, cutoff):
                rec.passes.append((time, p.id))
                del rec.behind_since[p.id]

    @staticmethod
    def _was_behind(rec: _AgentRecord, other: int, cutoff: float) -> bool:
        return rec.behind_since.get(other, -math.inf) >= cutoff


def progress_gained(memory: Memory, now: float, span: float) -> float:
    """Progress gained over the last ``span`` seconds before ``now``.

    Returns ``math.inf`` if the history does not cover the span, that is,
    if it is empty or its earliest entry is later than ``now - span``.
    """
    history = memory.progress_history
    if not history or history[0][0] > now - span:
        return math.inf
    recent = [p for t, p in history if t >= now - span]
    return recent[-1] - recent[0]


def passes_since(memory: Memory, since: float) -> int:
    """Number of recorded passes at or after time ``since``."""
    return sum(1 for t, _ in memory.passes if t >= since)
