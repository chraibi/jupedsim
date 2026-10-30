# SPDX-License-Identifier: LGPL-3.0-or-later

import dataclasses

import jupedsim as jps
import pytest
import shapely
from jupedsim.repertoires import (
    Capability,
    Command,
    Decision,
    RepertoireController,
    SelectorError,
    model_capabilities,
)
from jupedsim.repertoires.log import RunLog, load_records
from jupedsim.repertoires.perception import MemoryTracker, Perception

ROOM = shapely.box(0, 0, 10, 10)
BOTH = frozenset({Capability.TARGET_CONTROL, Capability.DESIRED_SPEED})


class StubRepertoire:
    """Walks towards a fixed target and counts control calls."""

    def __init__(
        self,
        name,
        target=(8.0, 5.0),
        requires=frozenset({Capability.TARGET_CONTROL}),
        event=None,
        handoff=False,
    ):
        self.name = name
        self.requires = requires
        self.target = target
        self.event = event
        self.handoff = handoff
        self.controls = {}

    def enter(self, obs, state):
        return dataclasses.replace(state, commitment_since=obs.time)

    def control(self, obs, state):
        self.controls[obs.agent_id] = self.controls.get(obs.agent_id, 0) + 1
        return Command(
            target=self.target,
            desired_speed=1.0,
            execution="walking",
            state=dataclasses.replace(state, target=self.target),
            event=self.event,
            handoff=self.handoff,
        )


class SwapSelector:
    """Moves every member to the other repertoire and records calls."""

    def __init__(self, sim=None):
        self.sim = sim
        self.calls = []

    def decide(self, requests):
        now = None if self.sim is None else self.sim.elapsed_time()
        self.calls.append((now, list(requests)))
        swap = {"a": "b", "b": "a"}
        return [
            Decision(r.unit_id, tuple(swap[s.repertoire] for s in r.states))
            for r in requests
        ]


class RaisingSelector:
    def decide(self, requests):
        raise RuntimeError("broken")


class EmptySelector:
    def decide(self, requests):
        return []


def make_sim():
    surface = jps.WalkableSurface()
    room = surface.add_region(polygon=ROOM)
    sim = jps.Simulation(
        model=jps.CollisionFreeSpeedModel(), geometry=surface, dt=0.01
    )
    steering = sim.add_direct_steering_stage()
    steering_journey = sim.add_journey(jps.JourneyDescription([steering]))
    exit_stage = sim.add_exit_stage(
        [([(9, 4), (10, 4), (10, 6), (9, 6)], room)]
    )
    exit_journey = sim.add_journey(jps.JourneyDescription([exit_stage]))
    return sim, room, (steering_journey, steering, exit_journey, exit_stage)


def align_agent_ids():
    """Make the next agent id end in 1.

    Agent ids are global across simulations and set each unit's phase
    offset ``(unit_id % 10) / 10 * decision_period``. Burning ids in a
    separate simulation gives the first agent of every test the same
    offset, 0.1 s, whatever ran before.
    """
    sim, room, (journey, stage, _, _) = make_sim()
    for k in range(10):
        agent_id = sim.add_agent(
            journey_id=journey,
            stage_id=stage,
            position=(0.5 + k, 0.5),
            state=jps.CollisionFreeSpeedModelState(),
            region_id=room,
        )
        if agent_id % 10 == 0:
            return


def add_agents(sim, room, ids, positions):
    steering_journey, steering, _, _ = ids
    align_agent_ids()
    return [
        sim.add_agent(
            journey_id=steering_journey,
            stage_id=steering,
            position=p,
            state=jps.CollisionFreeSpeedModelState(),
            region_id=room,
        )
        for p in positions
    ]


def make_controller(sim, ids, repertoires, selector, log, **overrides):
    steering_journey, steering, exit_journey, exit_stage = ids
    kwargs = dict(
        repertoires={r.name: r for r in repertoires},
        selector=selector,
        perception=Perception(
            progress=lambda p: p[0],
            radius=5.0,
            half_angle_deg=90.0,
            heading=lambda p: (1.0, 0.0),
        ),
        memory=MemoryTracker(window=10.0, pass_tolerance=0.2),
        log=log,
        steering_journey=steering_journey,
        steering_stage=steering,
        exit_journey=exit_journey,
        exit_stage=exit_stage,
        walkable=ROOM,
        control_interval=10,
        decision_period=1.0,
        capabilities=BOTH,
    )
    kwargs.update(overrides)
    return RepertoireController(sim, **kwargs)


POSITIONS = [(1.0, 2.0), (1.0, 5.0), (1.0, 8.0)]


def test_all_requests_share_one_snapshot_time(tmp_path):
    sim, room, ids = make_sim()
    agents = add_agents(sim, room, ids, POSITIONS)
    selector = SwapSelector(sim)
    reps = [StubRepertoire("a", event="tick"), StubRepertoire("b")]
    with RunLog(tmp_path / "log.jsonl", {}) as log:
        ctrl = make_controller(sim, ids, reps, selector, log)
        for agent_id in agents:
            ctrl.add(agent_id, "a")
        for _ in range(10):
            ctrl.step()
    assert any(len(reqs) > 1 for _, reqs in selector.calls)
    for now, reqs in selector.calls:
        assert {r.snapshot_time for r in reqs} == {now}


def relabel(record, first):
    """Agent ids are global across simulations; make them run-relative."""
    record = dict(record)
    for key in ("agent_id", "unit_id"):
        if key in record:
            record[key] -= first
    return record


def run_swaps(path, order):
    sim, room, ids = make_sim()
    agents = add_agents(sim, room, ids, POSITIONS)
    # Every unit is due every cycle, so phase offsets of ids do not matter.
    reps = [
        StubRepertoire("a", event="tick"),
        StubRepertoire("b", event="tock"),
    ]
    with RunLog(path, {}) as log:
        ctrl = make_controller(sim, ids, reps, SwapSelector(), log)
        for k in order:
            ctrl.add(agents[k], "a")
        for _ in range(20):
            ctrl.step()
    return [
        relabel(r, agents[0])
        for r in load_records(path)
        if r["type"] in ("transition", "response")
    ]


def test_registration_order_does_not_change_log(tmp_path):
    forward = run_swaps(tmp_path / "f.jsonl", [0, 1, 2])
    backward = run_swaps(tmp_path / "b.jsonl", [2, 1, 0])
    assert any(r["type"] == "transition" for r in forward)
    assert forward == backward


def test_control_runs_between_decisions(tmp_path):
    sim, room, ids = make_sim()
    agents = add_agents(sim, room, ids, POSITIONS)
    selector = SwapSelector()
    reps = [StubRepertoire("a"), StubRepertoire("b")]
    with RunLog(tmp_path / "log.jsonl", {}) as log:
        ctrl = make_controller(sim, ids, reps, selector, log)
        for agent_id in agents:
            ctrl.add(agent_id, "a")
        for _ in range(50):
            ctrl.step()
    for agent_id in agents:
        controls = sum(r.controls.get(agent_id, 0) for r in reps)
        requests = sum(
            1
            for _, reqs in selector.calls
            for r in reqs
            if r.unit_id == agent_id
        )
        assert controls == 50
        assert 1 <= requests <= 6


def test_missing_capability_rejected(tmp_path):
    sim, _, ids = make_sim()
    reps = [StubRepertoire("a", requires=BOTH)]
    with RunLog(tmp_path / "log.jsonl", {}) as log:
        with pytest.raises(ValueError):
            make_controller(
                sim,
                ids,
                reps,
                SwapSelector(),
                log,
                capabilities=frozenset({Capability.TARGET_CONTROL}),
            )


def test_model_capabilities_of_collision_free_speed_model():
    sim, room, ids = make_sim()
    (agent_id,) = add_agents(sim, room, ids, [(2.0, 5.0)])
    assert model_capabilities(sim.agent(agent_id).state) == BOTH
    assert model_capabilities(object()) == frozenset(
        {Capability.TARGET_CONTROL}
    )


def test_unwalkable_target_holds_position(tmp_path):
    sim, room, ids = make_sim()
    (agent_id,) = add_agents(sim, room, ids, [(2.0, 5.0)])
    reps = [StubRepertoire("a", target=(20.0, 20.0))]
    path = tmp_path / "log.jsonl"
    with RunLog(path, {}) as log:
        ctrl = make_controller(sim, ids, reps, SwapSelector(), log)
        ctrl.add(agent_id, "a")
        ctrl.step()
    target = sim.agent(agent_id).final_target
    assert (target.x, target.y) == pytest.approx((2.0, 5.0))
    assert ctrl.states[agent_id].execution == "standing"
    executions = [r for r in load_records(path) if r["type"] == "execution"]
    assert executions[-1]["execution"] == "standing"


def violation_raises(tmp_path, selector):
    sim, room, ids = make_sim()
    (agent_id,) = add_agents(sim, room, ids, [(2.0, 5.0)])
    reps = [StubRepertoire("a", event="tick")]
    with RunLog(tmp_path / "log.jsonl", {}) as log:
        ctrl = make_controller(sim, ids, reps, selector, log)
        ctrl.add(agent_id, "a")
        ctrl.step()
        before = sim.iteration_count()
        with pytest.raises(SelectorError):
            ctrl.step()
    assert sim.iteration_count() == before


def test_selector_contract_violation_raises(tmp_path):
    violation_raises(tmp_path, RaisingSelector())
    violation_raises(tmp_path, EmptySelector())


def test_handoff_switches_to_exit_journey(tmp_path):
    sim, room, ids = make_sim()
    (agent_id,) = add_agents(sim, room, ids, [(2.0, 5.0)])
    reps = [StubRepertoire("a", handoff=True)]
    with RunLog(tmp_path / "log.jsonl", {}) as log:
        ctrl = make_controller(sim, ids, reps, SwapSelector(), log)
        ctrl.add(agent_id, "a")
        ctrl.step()
    assert sim.agent(agent_id).journey_id == ids[2]
    assert agent_id not in ctrl.states


def test_handoff_is_logged_with_active_repertoire(tmp_path):
    sim, room, ids = make_sim()
    (agent_id,) = add_agents(sim, room, ids, [(2.0, 5.0)])
    reps = [StubRepertoire("a", handoff=True)]
    path = tmp_path / "log.jsonl"
    with RunLog(path, {}) as log:
        ctrl = make_controller(sim, ids, reps, SwapSelector(), log)
        ctrl.add(agent_id, "a")
        ctrl.step()
    handoffs = [r for r in load_records(path) if r["type"] == "handoff"]
    assert handoffs == [
        {
            "type": "handoff",
            "time": 0.0,
            "agent_id": agent_id,
            "unit_id": agent_id,
            "repertoire": "a",
        }
    ]


def test_adding_controlled_agent_twice_raises(tmp_path):
    sim, room, ids = make_sim()
    (agent_id,) = add_agents(sim, room, ids, [(2.0, 5.0)])
    with RunLog(tmp_path / "log.jsonl", {}) as log:
        ctrl = make_controller(
            sim, ids, [StubRepertoire("a")], SwapSelector(), log
        )
        ctrl.add(agent_id, "a")
        with pytest.raises(ValueError, match="already controlled"):
            ctrl.add(agent_id, "a")
