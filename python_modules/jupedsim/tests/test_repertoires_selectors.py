# SPDX-License-Identifier: LGPL-3.0-or-later

import pytest
from jupedsim.repertoires import (
    BehaviouralState,
    Decision,
    DecisionRequest,
    FixedSelector,
    ReplaySelector,
    SelectorError,
    sampling_seed,
    validate,
)
from jupedsim.repertoires.log import RunLog


def _state(repertoire: str, unit_id: int) -> BehaviouralState:
    return BehaviouralState(
        repertoire=repertoire,
        execution="x",
        anchor=None,
        target=None,
        unit_id=unit_id,
    )


def _request(
    unit_id: int, index: int = 0, repertoire: str = "a", unit_index=None
):
    return DecisionRequest(
        unit_id=unit_id,
        members=(unit_id,),
        snapshot_time=0.0,
        decision_index=index,
        observations=(),
        states=(_state(repertoire, unit_id),),
        events=(),
        unit_index=unit_id if unit_index is None else unit_index,
    )


def test_fixed_selector_keeps_repertoire():
    requests = [_request(1, repertoire="a"), _request(2, repertoire="b")]
    decisions = FixedSelector().decide(requests)
    assert decisions == [Decision(1, ("a",)), Decision(2, ("b",))]


def test_replay_returns_logged_decision(tmp_path):
    path = tmp_path / "log.jsonl"
    with RunLog(path, {}) as log:
        log.response(Decision(1, ("b",)), 0, 0.0, unit_index=1)
        log.response(Decision(1, ("c",)), 1, 1.0, unit_index=1)
        log.response(Decision(2, ("d",)), 0, 0.0, unit_index=2)
    selector = ReplaySelector(path)
    decisions = selector.decide([_request(2, 0), _request(1, 1)])
    assert decisions == [Decision(2, ("d",)), Decision(1, ("c",))]


def test_replay_matches_on_unit_index(tmp_path):
    path = tmp_path / "log.jsonl"
    with RunLog(path, {}) as log:
        log.response(Decision(101, ("b",)), 0, 0.0, unit_index=0)
    decisions = ReplaySelector(path).decide([_request(7, 0, unit_index=0)])
    assert decisions == [Decision(7, ("b",))]


def test_replay_restores_reasons(tmp_path):
    path = tmp_path / "log.jsonl"
    with RunLog(path, {}) as log:
        log.response(Decision(1, ("b",), ("why",)), 0, 0.0, unit_index=1)
    decisions = ReplaySelector(path).decide([_request(1, 0)])
    assert decisions == [Decision(1, ("b",), ("why",))]


def test_replay_missing_entry_raises(tmp_path):
    path = tmp_path / "log.jsonl"
    with RunLog(path, {}) as log:
        log.response(Decision(1, ("b",)), 0, 0.0, unit_index=1)
    with pytest.raises(SelectorError):
        ReplaySelector(path).decide([_request(1, 5)])


def test_validate_rejects_missing_extra_and_foreign_units():
    requests = [_request(1), _request(2)]
    validate(requests, [Decision(2, ("a",)), Decision(1, ("a",))])
    with pytest.raises(SelectorError):
        validate(requests, [Decision(1, ("a",))])
    with pytest.raises(SelectorError):
        validate(requests, [*map(lambda u: Decision(u, ("a",)), (1, 2, 2))])
    with pytest.raises(SelectorError):
        validate(requests, [Decision(1, ("a",)), Decision(3, ("a",))])
    with pytest.raises(SelectorError):
        validate(requests, [Decision(1, ("a",)), Decision(2, ("a", "a"))])


def test_sampling_seed_independent_of_request_order():
    keys = [(1, 0), (2, 0), (1, 1)]
    forward = {k: sampling_seed(7, *k) for k in keys}
    backward = {k: sampling_seed(7, *k) for k in reversed(keys)}
    assert forward == backward
    assert len(set(forward.values())) == len(keys)
    assert sampling_seed(8, 1, 0) != forward[(1, 0)]
