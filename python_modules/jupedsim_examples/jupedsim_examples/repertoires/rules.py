# SPDX-License-Identifier: LGPL-3.0-or-later
"""Rule-based selector for the entrance example.

The rules below are milestone-1 scientific assumptions, not validated
results: a queueing agent switches to overtaking when it is stalled and
a passing point exists, or when it has been passed repeatedly; an
overtaking agent returns to queueing once the overtake has completed or
been abandoned.
"""

import dataclasses
from collections.abc import Sequence

import shapely
from jupedsim.repertoires import (
    BehaviouralState,
    Decision,
    DecisionRequest,
    Observation,
)
from jupedsim.repertoires.perception import passes_since, progress_gained

from .entrance import EntranceParameters, Overtaking, Queueing, passing_point


@dataclasses.dataclass(frozen=True)
class RuleParameters:
    """Parameters of the entrance selection rules.

    Attributes:
        t_stall: Span over which progress is checked for a stall (s).
        stall_progress: Progress below which the agent counts as
            stalled (m).
        n_passes: Number of "passed me" events that trigger overtaking.
        t_passes: Time window for counting those events (s).
    """

    t_stall: float = 4.0
    stall_progress: float = 0.3
    n_passes: int = 2
    t_passes: float = 6.0


class EntranceRuleSelector:
    """Select queueing or overtaking from one agent's own observation."""

    def __init__(
        self,
        rules: RuleParameters,
        entrance: EntranceParameters,
        walkable: shapely.Polygon,
    ):
        self._rules = rules
        self._entrance = entrance
        self._walkable = walkable

    @property
    def rules(self) -> RuleParameters:
        """Parameters of the selection rules (read-only)."""
        return self._rules

    def decide(self, requests: Sequence[DecisionRequest]) -> Sequence[Decision]:
        """Return one decision per request, in request order."""
        return [Decision(r.unit_id, (self._select(r),)) for r in requests]

    def _select(self, req: DecisionRequest) -> str:
        current = req.states[0].repertoire
        if current == Queueing.name:
            return self._from_queueing(req)
        if current == Overtaking.name:
            return self._from_overtaking(req)
        return current

    def _from_queueing(self, req: DecisionRequest) -> str:
        obs, state = req.observations[0], req.states[0]
        if self._stalled_with_gap(obs, state) or self._passed_often(obs):
            return Overtaking.name
        return Queueing.name

    @staticmethod
    def _from_overtaking(req: DecisionRequest) -> str:
        finished = {"overtake_completed", "overtake_abandoned"}
        if finished.intersection(req.events):
            return Queueing.name
        return Overtaking.name

    def _stalled_with_gap(
        self, obs: Observation, state: BehaviouralState
    ) -> bool:
        gained = progress_gained(obs.memory, obs.time, self._rules.t_stall)
        if gained >= self._rules.stall_progress:
            return False
        anchor = next((p for p in obs.perceived if p.id == state.anchor), None)
        if anchor is None:
            return False
        point = passing_point(
            self._entrance,
            self._walkable,
            obs.position,
            anchor,
            obs.perceived,
        )
        return point is not None

    def _passed_often(self, obs: Observation) -> bool:
        since = obs.time - self._rules.t_passes
        return passes_since(obs.memory, since) >= self._rules.n_passes
