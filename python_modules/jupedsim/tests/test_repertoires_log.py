# SPDX-License-Identifier: LGPL-3.0-or-later

import json

from jupedsim.repertoires.log import RunLog, load_records, transitions_by_agent
from jupedsim.repertoires.types import (
    BehaviouralState,
    Decision,
    DecisionRequest,
    Memory,
    Observation,
)


def make_request():
    obs = Observation(
        agent_id=1,
        time=2.0,
        position=(0.0, 1.0),
        progress=3.0,
        perceived=(),
        memory=Memory((), (), False),
    )
    state = BehaviouralState("a", "x", None, None, unit_id=7)
    return DecisionRequest(7, (1,), 2.0, 0, (obs,), (state,), ("start",))


def test_first_record_is_metadata(tmp_path):
    path = tmp_path / "run.jsonl"
    with RunLog(path, {"seed": 3, "selector": "fixed"}):
        pass
    first = json.loads(path.read_text().splitlines()[0])
    assert first == {"type": "metadata", "seed": 3, "selector": "fixed"}


def test_transitions_by_agent_groups_and_sorts(tmp_path):
    path = tmp_path / "run.jsonl"
    with RunLog(path, {}) as log:
        log.transition(2.0, 5, 5, None, "a", "b", "event")
        log.transition(1.0, 6, 6, 2, "a", "b", "event")
        log.transition(1.0, 5, 5, None, "b", "a", "stall")
    result = transitions_by_agent(path)
    assert sorted(result) == [5, 6]
    assert [r["time"] for r in result[5]] == [1.0, 2.0]
    assert result[6][0]["group_id"] == 2


def test_requests_and_responses_round_trip(tmp_path):
    path = tmp_path / "run.jsonl"
    req = make_request()
    with RunLog(path, {}) as log:
        log.observation(req.observations[0])
        log.request(req)
        log.response(Decision(7, ("a",)), 4, 2.0)
        log.execution(1.0, 1, "a", "x")
        log.execution(1.5, 1, "a", "x")
        log.execution(2.0, 1, "a", "y")
        log.diagnostic(2.0, "cycle", {"ids": [1, 2]})
    records = load_records(path)
    by_type = {}
    for r in records:
        by_type.setdefault(r["type"], []).append(r)
    request = by_type["request"][0]
    assert request["unit_id"] == 7
    assert request["members"] == [1]
    assert request["snapshot_time"] == 2.0
    assert request["decision_index"] == 0
    assert request["events"] == ["start"]
    assert request["observations"][0]["memory"]["anchor_lost"] is False
    assert request["states"][0]["unit_id"] == 7
    assert by_type["observation"][0]["agent_id"] == 1
    response = by_type["response"][0]
    assert response["unit_id"] == 7
    assert response["repertoires"] == ["a"]
    assert response["decision_index"] == 4
    assert response["snapshot_time"] == 2.0
    assert [r["execution"] for r in by_type["execution"]] == ["x", "y"]
    assert by_type["diagnostic"][0]["detail"] == {"ids": [1, 2]}


def test_identical_runs_write_identical_files(tmp_path):
    for name in ("a", "b"):
        with RunLog(tmp_path / name, {"z": 1, "a": 2}) as log:
            log.request(make_request())
    assert (tmp_path / "a").read_bytes() == (tmp_path / "b").read_bytes()
