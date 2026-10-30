# SPDX-License-Identifier: LGPL-3.0-or-later
"""Rule-based selector for the entrance example.

The rules below are milestone-1 scientific assumptions, not validated
results. Entry into overtaking separates two questions:

- Motivation: the agent is stalled, or it has been passed repeatedly.
- Feasibility: a passing point beside its predecessor is currently
  perceived (the same check the overtaking repertoire uses while it
  executes, with the entry clearance ``entry_free_radius`` instead of
  the continuation clearance ``continuation_free_radius``).

A queueing agent starts overtaking only when it is motivated and the
manoeuvre is feasible. A motivated agent without a passing point keeps
queueing; the decision carries the reason ``overtake_withheld_no_gap``.
Motivation lasts as long as its memory window, so the agent enters
overtaking if a gap opens while that condition still holds. An
overtaking agent returns to queueing once the overtake has completed or
been abandoned, whatever the abandonment reason.
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

WITHHELD_NO_GAP = "overtake_withheld_no_gap"


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
        return [self._select(r) for r in requests]

    def _select(self, req: DecisionRequest) -> Decision:
        current = req.states[0].repertoire
        if current == Queueing.name:
            return self._from_queueing(req)
        if current == Overtaking.name:
            return Decision(req.unit_id, (self._from_overtaking(req),))
        return Decision(req.unit_id, (current,))

    def _from_queueing(self, req: DecisionRequest) -> Decision:
        obs, state = req.observations[0], req.states[0]
        if not (self._stalled(obs) or self._passed_often(obs)):
            return Decision(req.unit_id, (Queueing.name,))
        if not self._gap_exists(obs, state):
            return Decision(req.unit_id, (Queueing.name,), (WITHHELD_NO_GAP,))
        return Decision(req.unit_id, (Overtaking.name,))

    @staticmethod
    def _from_overtaking(req: DecisionRequest) -> str:
        finished = any(
            e == "overtake_completed" or e.startswith("overtake_abandoned")
            for e in req.events
        )
        return Queueing.name if finished else Overtaking.name

    def _stalled(self, obs: Observation) -> bool:
        gained = progress_gained(obs.memory, obs.time, self._rules.t_stall)
        return gained < self._rules.stall_progress

    def _gap_exists(self, obs: Observation, state: BehaviouralState) -> bool:
        anchor = next((p for p in obs.perceived if p.id == state.anchor), None)
        if anchor is None:
            return False
        point = passing_point(
            self._entrance,
            self._walkable,
            obs.position,
            anchor,
            obs.perceived,
            self._entrance.entry_free_radius,
        )
        return point is not None

    def _passed_often(self, obs: Observation) -> bool:
        since = obs.time - self._rules.t_passes
        return passes_since(obs.memory, since) >= self._rules.n_passes
