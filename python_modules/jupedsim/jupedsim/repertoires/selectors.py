# SPDX-License-Identifier: LGPL-3.0-or-later

import pathlib
from collections.abc import Sequence

import numpy as np

from jupedsim.repertoires.log import load_records
from jupedsim.repertoires.types import (
    Decision,
    DecisionRequest,
    SelectorError,
)


class FixedSelector:
    """Keeps every member in its current repertoire."""

    def decide(self, requests: Sequence[DecisionRequest]) -> list[Decision]:
        """Return each member's current repertoire."""
        return [
            Decision(req.unit_id, tuple(s.repertoire for s in req.states))
            for req in requests
        ]


class ReplaySelector:
    """Answers requests from the responses recorded in a run log."""

    def __init__(self, path: pathlib.Path):
        """Index the logged responses by unit index and decision index.

        Keys use the run-relative unit index, so a log can be replayed in
        a simulation whose agent ids differ.

        Args:
            path: Log file written by ``RunLog``.
        """
        self._responses = {
            (r["unit_index"], r["decision_index"]): tuple(r["repertoires"])
            for r in load_records(path)
            if r["type"] == "response"
        }

    def decide(self, requests: Sequence[DecisionRequest]) -> list[Decision]:
        """Return the logged decision for each request.

        Raises:
            SelectorError: If a request has no logged response.
        """
        return [self._lookup(req) for req in requests]

    def _lookup(self, req: DecisionRequest) -> Decision:
        key = (req.unit_index, req.decision_index)
        if key not in self._responses:
            raise SelectorError(
                f"no logged response for unit index {req.unit_index}, "
                f"decision {req.decision_index}"
            )
        return Decision(req.unit_id, self._responses[key])


def validate(
    requests: Sequence[DecisionRequest], decisions: Sequence[Decision]
) -> None:
    """Check that decisions match requests one-to-one.

    Raises:
        SelectorError: If units are missing, repeated or foreign, or a
            decision does not name one repertoire per member.
    """
    units = [d.unit_id for d in decisions]
    expected = {req.unit_id: len(req.members) for req in requests}
    if len(units) != len(set(units)) or set(units) != set(expected):
        raise SelectorError(
            f"decision units {sorted(units)} do not match request units "
            f"{sorted(expected)}"
        )
    for dec in decisions:
        if len(dec.repertoires) != expected[dec.unit_id]:
            raise SelectorError(
                f"unit {dec.unit_id}: expected {expected[dec.unit_id]} "
                f"repertoires, got {len(dec.repertoires)}"
            )


def sampling_seed(run_seed: int, unit_id: int, decision_index: int) -> int:
    """Return a seed that depends only on its arguments.

    The result does not depend on the order in which requests are
    processed.
    """
    sequence = np.random.SeedSequence([run_seed, unit_id, decision_index])
    return int(sequence.generate_state(1)[0])
