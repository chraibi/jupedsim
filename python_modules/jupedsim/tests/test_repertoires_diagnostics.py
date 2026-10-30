# SPDX-License-Identifier: LGPL-3.0-or-later

import dataclasses

import jupedsim as jps
import shapely
from jupedsim.repertoires import (
    Capability,
    Command,
    Decision,
    RepertoireController,
    anchor_cycles,
    anchor_structure,
)
from jupedsim.repertoires.log import RunLog, load_records
from jupedsim.repertoires.perception import MemoryTracker, Perception

ROOM = shapely.box(0, 0, 10, 10)
BOTH = frozenset({Capability.TARGET_CONTROL, Capability.DESIRED_SPEED})
POSITIONS = [(1.0, 2.0), (1.0, 5.0)]


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


def add_agents(sim, room, ids, positions):
    steering_journey, steering, _, _ = ids
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


def test_detects_two_and_three_cycles():
    anchors = {1: 2, 2: 1, 3: 4, 4: 5, 5: 3, 6: 1, 7: None, 8: 99, 9: 9}
    assert anchor_cycles(anchors) == [(1, 2), (3, 4, 5), (9,)]


def test_shared_predecessor_is_not_a_cycle():
    anchors = {1: 0, 2: 0, 0: None}
    assert anchor_cycles(anchors) == []
    assert anchor_structure(anchors) == {
        "anchored": 2,
        "unanchored": 1,
        "shared": 1,
    }


class CyclicRepertoire:
    """Anchors each agent to the next id in a ring of the first two."""

    name = "ring"
    requires = frozenset()

    def __init__(self, ring):
        self.ring = ring

    def enter(self, obs, state):
        other = [a for a in self.ring if a != obs.agent_id]
        return dataclasses.replace(state, anchor=other[0])

    def control(self, obs, state):
        return Command(
            target=(8.0, 5.0),
            desired_speed=1.0,
            execution="walking",
            state=state,
        )


class KeepSelector:
    def decide(self, requests):
        return [
            Decision(r.unit_id, tuple(s.repertoire for s in r.states))
            for r in requests
        ]


def test_diagnostics_not_visible_to_selector(tmp_path):
    sim, room, ids = make_sim()
    agents = add_agents(sim, room, ids, POSITIONS)
    path = tmp_path / "log.jsonl"
    with RunLog(path, {}) as log:
        ctrl = make_controller(
            sim,
            ids,
            [CyclicRepertoire(agents)],
            KeepSelector(),
            log,
        )
        for agent_id in agents:
            ctrl.add(agent_id, "ring")
        for _ in range(3):
            ctrl.step()
    records = load_records(path)
    cycles = [
        r
        for r in records
        if r["type"] == "diagnostic" and r["kind"] == "anchor_cycle"
    ]
    assert cycles and cycles[0]["detail"] == {"members": sorted(agents)}
    requests = [r for r in records if r["type"] == "request"]
    assert requests
    for record in requests:
        assert "anchor_cycle" not in str(record)
        assert "cycle" not in record
