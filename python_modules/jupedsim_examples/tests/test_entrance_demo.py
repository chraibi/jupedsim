# SPDX-License-Identifier: LGPL-3.0-or-later
import math

import jupedsim as jps
from jupedsim.repertoires.log import load_records, transitions_by_agent
from jupedsim_examples.repertoires import EntranceRuleSelector, RuleParameters
from jupedsim_examples.repertoires.demo import (
    ARRIVALS,
    CORRIDOR,
    DEMO_ENTRANCE,
    STANDING,
    build_demo,
    run_demo,
)


def test_arrivals_overtake_the_standing_person(tmp_path):
    log = tmp_path / "demo.jsonl"
    sim, controller, standing = build_demo(
        jps.CollisionFreeSpeedModel(),
        selector=EntranceRuleSelector(
            RuleParameters(), DEMO_ENTRANCE, CORRIDOR
        ),
        log_path=log,
    )
    frames = []
    run_demo(
        sim, controller, 60.0, lambda s, c: frames.append(s.elapsed_time())
    )
    assert [a.id for a in sim.agents()] == [standing]
    assert math.dist(sim.agent(standing).position, STANDING) < 0.05
    assert len(frames) > 0
    causes = [
        r["cause"]
        for records in transitions_by_agent(log).values()
        for r in records
    ]
    assert "overtake_completed" in causes
    handoffs = [r for r in load_records(log) if r["type"] == "handoff"]
    assert len(handoffs) == len(ARRIVALS)
