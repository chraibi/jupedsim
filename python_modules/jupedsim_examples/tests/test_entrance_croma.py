# SPDX-License-Identifier: LGPL-3.0-or-later
import dataclasses

import jupedsim as jps
import pytest
import shapely
from jupedsim.repertoires import FixedSelector
from jupedsim.repertoires.log import load_records
from jupedsim_examples.repertoires import EntranceRuleSelector, RuleParameters
from jupedsim_examples.repertoires.croma import (
    ENTRANCE,
    Entry,
    build_2c120,
    corridor,
    geometry,
    load_entries,
    run_2c120,
)

PETRACK = """\
# framerate: 50 fps
# id frame x/m y/m z/m markerID
2\t110\t1.0\t2.0\t1.8\t7
2\t111\t1.1\t2.1\t1.8\t7
1\t100\t-1.0\t-0.5\t1.7\t5
1\t101\t-1.0\t-0.4\t1.7\t5
3\t100\t0.5\t-0.9\t1.7\t6
"""

OPEN = 4.0


def test_load_entries_takes_first_frame_per_person(tmp_path):
    path = tmp_path / "petrack.txt"
    path.write_text(PETRACK)
    assert load_entries(path) == [
        Entry(person_id=1, time=0.0, position=(-1.0, -0.5)),
        Entry(person_id=3, time=0.0, position=(0.5, -0.9)),
        Entry(person_id=2, time=0.2, position=(1.0, 2.0)),
    ]


def test_geometry_closes_the_door():
    hall = geometry()
    assert hall.is_valid
    assert hall.covers(corridor())
    assert not hall.covers(shapely.Point(0.0, 20.5))
    assert hall.covers(shapely.Point(0.0, 22.5))


def entries():
    return [
        Entry(1, 0.0, (0.0, 15.0)),
        Entry(2, 0.0, (1.0, 14.0)),
        Entry(3, 0.5, (-1.0, 14.0)),
    ]


def build(tmp_path, entries, selector=None):
    return build_2c120(
        jps.CollisionFreeSpeedModelV3(),
        entries=entries,
        selector=selector or FixedSelector(),
        log_path=tmp_path / "log.jsonl",
        entrance=dataclasses.replace(ENTRANCE, door_open_time=OPEN),
    )


def step_until(sim, controller, inflow, time):
    while sim.elapsed_time() < time:
        inflow.admit(sim, controller)
        controller.step()


def test_agents_enter_at_their_times(tmp_path):
    sim, controller, inflow = build(tmp_path, entries())
    step_until(sim, controller, inflow, 0.3)
    assert sorted(inflow.added) == [1, 2]
    step_until(sim, controller, inflow, 0.6)
    assert sorted(inflow.added) == [1, 2, 3]
    assert inflow.pending == 0


def test_occupied_entry_waits(tmp_path):
    same = [Entry(1, 0.0, (0.0, 10.0)), Entry(2, 0.0, (0.0, 10.0))]
    sim, controller, inflow = build(tmp_path, same)
    step_until(sim, controller, inflow, 0.1)
    assert list(inflow.added) == [1]
    step_until(sim, controller, inflow, 2.0)
    assert sorted(inflow.added) == [1, 2]


def test_nobody_leaves_before_the_door_opens(tmp_path):
    sim, controller, inflow = build(tmp_path, entries())
    step_until(sim, controller, inflow, OPEN - 0.1)
    assert sim.agent_count() == 3
    assert max(a.position[1] for a in sim.agents()) < 20.0
    run_2c120(sim, controller, inflow, 30.0)
    assert sim.agent_count() == 0
    handoffs = [
        r
        for r in load_records(tmp_path / "log.jsonl")
        if r["type"] == "handoff"
    ]
    assert len(handoffs) == 3
    assert min(r["time"] for r in handoffs) >= OPEN


def test_metadata_records_scenario(tmp_path):
    selector = EntranceRuleSelector(RuleParameters(), ENTRANCE, corridor())
    sim, controller, inflow = build(tmp_path, entries(), selector)
    run_2c120(sim, controller, inflow, 0.1)
    metadata = load_records(tmp_path / "log.jsonl")[0]
    assert metadata["scenario"] == "2C120"
    assert metadata["n_agents"] == 3
    assert metadata["entrance"]["door_open_time"] == OPEN
    assert metadata["rules"] == dataclasses.asdict(RuleParameters())


def test_unsupported_model_is_rejected(tmp_path):
    with pytest.raises(ValueError, match="unsupported model"):
        build_2c120(
            jps.GeneralizedCentrifugalForceModel(),
            entries=entries(),
            selector=FixedSelector(),
            log_path=tmp_path / "log.jsonl",
        )
