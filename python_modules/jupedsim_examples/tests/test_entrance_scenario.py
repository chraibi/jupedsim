# SPDX-License-Identifier: LGPL-3.0-or-later
import dataclasses
import json

import jupedsim as jps
import pytest
import shapely
from jupedsim.repertoires import FixedSelector, ReplaySelector
from jupedsim.repertoires.log import load_records, transitions_by_agent
from jupedsim_examples.repertoires import (
    EntranceParameters,
    EntranceRuleSelector,
    RuleParameters,
)
from jupedsim_examples.repertoires.scenario import (
    build_entrance,
    run,
    walkable_area,
)

N_AGENTS = 30
SEED = 1
MAX_TIME = 120.0


def align_agent_ids() -> int:
    """Make the next agent id end in 1.

    Agent ids are global across simulations and set each unit's phase
    offset ``(unit_id % 10) / 10 * decision_period``. Burning ids in a
    separate simulation gives runs compared with each other the same
    phase offsets.

    Returns:
        The next agent id.
    """
    surface = jps.WalkableSurface()
    room = surface.add_region(polygon=shapely.box(0, 0, 20, 2))
    sim = jps.Simulation(
        model=jps.CollisionFreeSpeedModel(), geometry=surface, dt=0.01
    )
    steering = sim.add_direct_steering_stage()
    journey = sim.add_journey(jps.JourneyDescription([steering]))
    for k in range(10):
        agent_id = sim.add_agent(
            journey_id=journey,
            stage_id=steering,
            position=(0.5 + k, 0.5),
            state=jps.CollisionFreeSpeedModelState(),
            region_id=room,
        )
        if agent_id % 10 == 0:
            return agent_id + 1


def rule_selector():
    return EntranceRuleSelector(
        RuleParameters(), EntranceParameters(), walkable_area()
    )


def run_scenario(model, selector, log_path, aligned=False):
    if not aligned:
        align_agent_ids()
    sim, controller = build_entrance(
        model,
        n_agents=N_AGENTS,
        seed=SEED,
        selector=selector,
        log_path=log_path,
    )
    ids = sorted(controller.states)
    run(sim, controller, MAX_TIME)
    return sim, ids


def transition_counts(path):
    counts = {}
    for records in transitions_by_agent(path).values():
        for r in records:
            counts[r["after"]] = counts.get(r["after"], 0) + 1
    return counts


@pytest.fixture(scope="module")
def rules_run(tmp_path_factory):
    log_path = tmp_path_factory.mktemp("rules") / "rules.jsonl"
    sim, ids = run_scenario(
        jps.CollisionFreeSpeedModel(), rule_selector(), log_path
    )
    return sim, ids, log_path


def test_all_agents_reach_entrance(rules_run):
    sim, _, _ = rules_run
    assert sim.agent_count() == 0


@pytest.mark.xfail(
    strict=True,
    reason=(
        "Milestone-1 rules: no overtake returns to queueing with "
        "CollisionFreeSpeedModel; awaiting maintainer decision on rules "
        "4(b)/6"
    ),
)
def test_rules_produce_queueing_and_overtaking(rules_run):
    _, _, log_path = rules_run
    counts = transition_counts(log_path)
    assert counts.get("overtaking", 0) >= 1
    assert counts.get("queueing", 0) >= 1


def test_fixed_selector_never_transitions(tmp_path):
    log_path = tmp_path / "fixed.jsonl"
    run_scenario(jps.CollisionFreeSpeedModel(), FixedSelector(), log_path)
    assert transitions_by_agent(log_path) == {}


@pytest.mark.parametrize(
    "model", [jps.CollisionFreeSpeedModel, jps.CollisionFreeSpeedModelV3]
)
def test_runs_with_both_movement_models(model, tmp_path):
    log_path = tmp_path / "model.jsonl"
    sim, _ = run_scenario(model(), rule_selector(), log_path)
    metadata = load_records(log_path)[0]
    assert metadata["model"] == model.__name__
    assert sim.agent_count() == 0


def shift_units(src, dst, offset):
    """Copy a log, shifting the unit id of every response record."""
    with open(dst, "w", encoding="utf-8") as file:
        for record in load_records(src):
            if record["type"] == "response":
                record = {**record, "unit_id": record["unit_id"] + offset}
            file.write(json.dumps(record, sort_keys=True) + "\n")


def normalised(path, ids):
    """Transitions keyed and labelled by each agent's order of addition."""
    index = {agent_id: k for k, agent_id in enumerate(ids)}
    return {
        index[agent_id]: [
            {
                **r,
                "agent_id": index[r["agent_id"]],
                "unit_id": index[r["unit_id"]],
            }
            for r in records
        ]
        for agent_id, records in transitions_by_agent(path).items()
    }


def test_replay_reproduces_transitions(tmp_path):
    # CollisionFreeSpeedModelV3 gives many transitions under the rules,
    # so the comparison below covers queueing and overtaking alike.
    log_a = tmp_path / "rules.jsonl"
    _, ids_a = run_scenario(
        jps.CollisionFreeSpeedModelV3(), rule_selector(), log_a
    )
    shifted = tmp_path / "shifted.jsonl"
    log_b = tmp_path / "replay.jsonl"
    offset = align_agent_ids() - ids_a[0]
    shift_units(log_a, shifted, offset)
    _, ids_b = run_scenario(
        jps.CollisionFreeSpeedModelV3(),
        ReplaySelector(shifted),
        log_b,
        aligned=True,
    )
    assert ids_b[0] - ids_a[0] == offset
    meta_a, meta_b = load_records(log_a)[0], load_records(log_b)[0]
    assert (meta_a["seed"], meta_a["model"]) == (
        meta_b["seed"],
        meta_b["model"],
    )
    expected = normalised(log_a, ids_a)
    assert sum(len(records) for records in expected.values()) > 10
    assert normalised(log_b, ids_b) == expected


def test_metadata_records_run(rules_run):
    _, _, log_path = rules_run
    metadata = load_records(log_path)[0]
    assert metadata["selector"] == "EntranceRuleSelector"
    assert metadata["seed"] == SEED
    assert metadata["n_agents"] == N_AGENTS
    assert metadata["model"] == "CollisionFreeSpeedModel"
    assert metadata["entrance"] == json.loads(
        json.dumps(dataclasses.asdict(EntranceParameters()))
    )
    assert metadata["control_interval"] == 10
    assert metadata["decision_period"] == 2.0
    assert metadata["rules"] == dataclasses.asdict(RuleParameters())
    assert metadata["jupedsim_version"] == jps.__version__
    assert metadata["perception"] == {"radius": 4.0, "half_angle_deg": 90.0}
    assert metadata["memory"] == {"window": 8.0, "pass_tolerance": 0.2}


def test_metadata_omits_rules_of_other_selectors(tmp_path):
    log_path = tmp_path / "fixed.jsonl"
    build_entrance(
        jps.CollisionFreeSpeedModel(),
        n_agents=2,
        seed=SEED,
        selector=FixedSelector(),
        log_path=log_path,
    )
    assert "rules" not in load_records(log_path)[0]


def test_rule_selector_exposes_parameters():
    rules = RuleParameters(t_stall=3.0)
    selector = EntranceRuleSelector(
        rules, EntranceParameters(), walkable_area()
    )
    assert selector.rules == rules


@pytest.mark.parametrize(
    "selector, entrance",
    [
        (FixedSelector(), EntranceParameters(abandon_span=10.0)),
        (
            EntranceRuleSelector(
                RuleParameters(t_passes=8.0),
                EntranceParameters(),
                walkable_area(),
            ),
            EntranceParameters(),
        ),
    ],
)
def test_memory_window_must_cover_queried_spans(selector, entrance, tmp_path):
    with pytest.raises(ValueError, match="memory window"):
        build_entrance(
            jps.CollisionFreeSpeedModel(),
            n_agents=2,
            seed=SEED,
            selector=selector,
            log_path=tmp_path / "log.jsonl",
            entrance=entrance,
        )
