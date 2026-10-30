# SPDX-License-Identifier: LGPL-3.0-or-later

import jupedsim as jps
import pytest
from jupedsim.repertoires.perception import (
    MemoryTracker,
    Perception,
    passes_since,
    progress_gained,
)
from jupedsim.repertoires.types import Memory, PerceivedAgent


def others(*rel, own=0.0):
    return [PerceivedAgent(i, (0.0, 0.0), own + r) for i, r in rel]


def test_behind_to_ahead_is_a_pass():
    tracker = MemoryTracker(window=10.0, pass_tolerance=0.2)
    tracker.update(0, 0.0, 0.0, others((1, -0.5)))
    mem = tracker.update(0, 0.1, 0.0, others((1, 0.5)))
    assert passes_since(mem, 0.0) == 1
    assert mem.passes == ((0.1, 1),)


def test_jitter_within_tolerance_is_not_a_pass():
    tracker = MemoryTracker(window=10.0, pass_tolerance=0.2)
    mem = None
    for k in range(20):
        r = 0.15 if k % 2 else -0.15
        mem = tracker.update(0, 0.1 * k, 0.0, others((1, r)))
    assert passes_since(mem, 0.0) == 0


def test_reappearance_ahead_is_not_a_pass():
    tracker = MemoryTracker(window=10.0, pass_tolerance=0.2)
    tracker.update(0, 0.0, 0.0, others((1, -0.5)))
    tracker.update(0, 0.1, 0.0, [])
    mem = tracker.update(0, 0.2, 0.0, others((1, 0.5)))
    assert passes_since(mem, 0.0) == 0


def test_progress_gained_over_span():
    mem = Memory(((0.0, -10.0), (2.0, -9.5), (4.0, -9.4)), (), False)
    assert progress_gained(mem, 4.0, 4.0) == pytest.approx(0.6)


def test_forget_drops_departed_agent():
    tracker = MemoryTracker(window=10.0, pass_tolerance=0.2)
    tracker.update(0, 0.0, 0.0, others((1, -0.5)))
    tracker.forget(1)
    mem = tracker.update(0, 0.1, 0.0, others((1, 0.5)))
    assert passes_since(mem, 0.0) == 0
    tracker.update(0, 0.2, 0.0, [])
    assert tracker._records[0].behind_since == {}


def test_old_history_and_passes_are_dropped():
    tracker = MemoryTracker(window=1.0, pass_tolerance=0.2)
    tracker.update(0, 0.0, 0.0, others((1, -0.5)))
    tracker.update(0, 0.1, 0.0, others((1, 0.5)))
    mem = tracker.update(0, 2.0, 0.0, [])
    assert mem.passes == ()
    assert mem.progress_history == ((2.0, 0.0),)


def test_anchor_lost_flag_until_cleared():
    tracker = MemoryTracker(window=10.0, pass_tolerance=0.2)
    assert not tracker.update(0, 0.0, 0.0, []).anchor_lost
    tracker.mark_anchor_lost(0)
    assert tracker.update(0, 0.1, 0.0, []).anchor_lost
    assert tracker.update(0, 0.2, 0.0, []).anchor_lost
    tracker.clear_anchor_lost(0)
    assert not tracker.update(0, 0.3, 0.0, []).anchor_lost


def test_view_cone_excludes_agents_behind():
    surface = jps.WalkableSurface()
    room = surface.add_region(boundary=[(0, 0), (10, 0), (10, 10), (0, 10)])
    sim = jps.Simulation(
        model=jps.CollisionFreeSpeedModel(), geometry=surface, dt=0.01
    )
    steering = sim.add_direct_steering_stage()
    journey = sim.add_journey(jps.JourneyDescription([steering]))
    ids = [
        sim.add_agent(
            journey_id=journey,
            stage_id=steering,
            position=pos,
            state=jps.CollisionFreeSpeedModelState(),
            region_id=room,
        )
        for pos in [(5.0, 5.0), (4.0, 5.0), (6.0, 5.0)]
    ]
    perception = Perception(
        progress=lambda pos: pos[0],
        radius=3.0,
        half_angle_deg=90.0,
        heading=lambda pos: (1.0, 0.0),
    )
    mem = Memory((), (), False)
    obs = perception.observe(sim, ids[0], 0.0, mem)
    assert [p.id for p in obs.perceived] == [ids[2]]
    assert obs.progress == pytest.approx(5.0)
    assert obs.perceived[0].progress == pytest.approx(6.0)
