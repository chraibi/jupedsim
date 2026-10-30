# SPDX-License-Identifier: LGPL-3.0-or-later

import dataclasses
import math

import pytest
import shapely
from jupedsim.repertoires import (
    BehaviouralState,
    Memory,
    Observation,
    PerceivedAgent,
)
from jupedsim_examples.repertoires import (
    EntranceParameters,
    Overtaking,
    Queueing,
    heading,
    passing_point,
    progress,
)

PARAMS = EntranceParameters()
WALKABLE = shapely.Polygon([(0, 0), (20, 0), (20, 10), (0, 10)])
EMPTY = Memory(progress_history=(), passes=(), anchor_lost=False)


def agent(agent_id, pos):
    return PerceivedAgent(agent_id, pos, progress(PARAMS)(pos))


def obs(pos, others=(), time=10.0, memory=EMPTY, agent_id=0):
    return Observation(
        agent_id=agent_id,
        time=time,
        position=pos,
        progress=progress(PARAMS)(pos),
        perceived=tuple(agent(i, p) for i, p in others),
        memory=memory,
    )


def state(repertoire="queueing", **kw):
    base = dict(
        repertoire=repertoire, execution="standing", anchor=None, target=None
    )
    return BehaviouralState(**{**base, **kw})


def test_progress_and_heading():
    assert progress(PARAMS)((17.0, 9.0)) == -5.0
    assert heading(PARAMS)((10.0, 5.0)) == (1.0, 0.0)
    assert heading(PARAMS)((20.0, 5.0)) == (1.0, 0.0)
    assert heading(PARAMS)((17.0, 9.0)) == (0.6, -0.8)


def test_predecessor_is_nearest_agent_ahead():
    o = obs((10, 5), [(1, (12, 5)), (2, (11, 6)), (3, (9, 5))])
    s = Queueing(PARAMS, WALKABLE).enter(o, state())
    assert s.anchor == 2
    assert s.phase == 0
    assert s.commitment_since == 10.0


def test_first_agent_targets_entrance():
    o = obs((10, 5), [(3, (9, 5))])
    q = Queueing(PARAMS, WALKABLE)
    s = q.enter(o, state())
    assert s.anchor is None
    cmd = q.control(o, s)
    assert cmd.target == (20.0, 5.0)
    assert cmd.execution == "advancing"
    assert cmd.desired_speed == 1.0


def test_queue_target_is_spacing_behind_predecessor():
    o = obs((10, 5), [(1, (15, 5))])
    q = Queueing(PARAMS, WALKABLE)
    cmd = q.control(o, q.enter(o, state()))
    assert cmd.state.anchor == 1
    assert math.isclose(cmd.target[0], 14.3)
    assert math.isclose(cmd.target[1], 5.0)
    assert cmd.execution == "advancing"


def test_standing_within_tolerance():
    o = obs((14.2, 5), [(1, (15, 5))])
    q = Queueing(PARAMS, WALKABLE)
    cmd = q.control(o, q.enter(o, state()))
    assert cmd.execution == "standing"


def test_queueing_reacquires_when_predecessor_leaves():
    lost = Memory((), (), anchor_lost=True)
    o = obs((10, 5), [(7, (13, 5))], memory=lost)
    q = Queueing(PARAMS, WALKABLE)
    cmd = q.control(o, state(anchor=1, target=(14.3, 5.0)))
    assert cmd.state.anchor == 7
    o2 = obs((10, 5), [], memory=lost)
    assert q.control(o2, state(anchor=1)).state.anchor is None


def test_queueing_reacquires_when_not_perceived_or_not_ahead():
    q = Queueing(PARAMS, WALKABLE)
    o = obs((10, 5), [(7, (13, 5))])
    assert q.control(o, state(anchor=1)).state.anchor == 7
    o = obs((10, 5), [(1, (9, 5)), (7, (13, 5))])
    assert q.control(o, state(anchor=1)).state.anchor == 7


def test_queueing_keeps_anchor_when_valid():
    q = Queueing(PARAMS, WALKABLE)
    o = obs((10, 5), [(1, (15, 5)), (7, (12, 5))])
    assert q.control(o, state(anchor=1)).state.anchor == 1


def test_handoff_near_entrance():
    q = Queueing(PARAMS, WALKABLE)
    o = obs((19.5, 5), [])
    assert q.control(o, state()).handoff
    ot = Overtaking(PARAMS, WALKABLE)
    assert ot.control(o, state("overtaking", anchor=1)).handoff
    assert not q.control(obs((18.9, 5), []), state()).handoff


def test_passing_point_prefers_closer_free_side():
    own, pred = (14.0, 5.3), agent(1, (15.0, 5.0))
    pp = passing_point(PARAMS, WALKABLE, own, pred, [pred])
    assert pp is not None
    assert math.isclose(pp[0], 15.0) and math.isclose(pp[1], 5.8)
    block = agent(2, (15.0, 5.8))
    pp = passing_point(PARAMS, WALKABLE, own, pred, [pred, block])
    assert math.isclose(pp[1], 4.2)
    block2 = agent(3, (15.0, 4.2))
    assert (
        passing_point(PARAMS, WALKABLE, own, pred, [pred, block, block2])
        is None
    )


def test_passing_point_tie_goes_to_positive_normal():
    pred = agent(1, (15.0, 5.0))
    pp = passing_point(PARAMS, WALKABLE, (14.0, 5.0), pred, [pred])
    assert math.isclose(pp[1], 5.8)


def test_passing_point_respects_wall_clearance():
    pred = agent(1, (15.0, 0.8))
    own = (16.0, 0.2)
    pp = passing_point(PARAMS, WALKABLE, own, pred, [pred])
    assert pp == pytest.approx((14.4854, 1.4126), abs=1e-3)
    bare = dataclasses.replace(PARAMS, wall_clearance=0.0)
    pp = passing_point(bare, WALKABLE, own, pred, [pred])
    assert pp == pytest.approx((15.5146, 0.1874), abs=1e-3)


def overtaking_state(**kw):
    base = dict(anchor=1, phase=1, commitment_since=9.0, execution="passing")
    return state("overtaking", **{**base, **kw})


def test_overtake_enter_sets_phase_and_commitment():
    o = obs((10, 5), [(1, (15, 5))])
    s = Overtaking(PARAMS, WALKABLE).enter(o, state(anchor=1))
    assert (s.repertoire, s.anchor, s.phase, s.commitment_since) == (
        "overtaking",
        1,
        1,
        10.0,
    )


def test_overtake_steers_to_lateral_point_then_ahead():
    ot = Overtaking(PARAMS, WALKABLE)
    o = obs((14.0, 5.3), [(1, (15, 5))])
    cmd = ot.control(o, overtaking_state())
    assert cmd.target == (15.0, 5.8)
    assert cmd.execution == "passing"
    assert cmd.desired_speed == 1.4
    assert cmd.state.phase == 1
    o = obs((14.9, 5.7), [(1, (15, 5))])
    cmd = ot.control(o, overtaking_state())
    assert cmd.state.phase == 2
    assert math.isclose(cmd.target[0], 15.8) and cmd.target[1] == 5.0


def test_overtake_completes_past_target():
    ot = Overtaking(PARAMS, WALKABLE)
    o = obs((15.5, 5.8), [(1, (15, 5))])
    cmd = ot.control(o, overtaking_state(phase=2))
    assert cmd.event == "overtake_completed"
    assert cmd.target == (20.0, 5.0)
    assert cmd.execution == "advancing"


def test_overtake_abandons_when_target_lost():
    ot = Overtaking(PARAMS, WALKABLE)
    cmd = ot.control(obs((14, 5), []), overtaking_state())
    assert cmd.event == "overtake_abandoned_target_lost"
    assert cmd.target == (14, 5)
    assert cmd.execution == "standing"
    cmd = ot.control(obs((14, 5), []), overtaking_state(anchor=None))
    assert cmd.event == "overtake_abandoned_target_lost"


def test_overtake_abandons_when_stalled():
    ot = Overtaking(PARAMS, WALKABLE)
    mem = Memory(((8.0, -6.0), (9.0, -6.0), (10.0, -6.0)), (), False)
    o = obs((14.0, 5.3), [(1, (15, 5))], time=10.0, memory=mem)
    cmd = ot.control(o, overtaking_state(commitment_since=8.0))
    assert cmd.event == "overtake_abandoned_stalled"
    young = ot.control(o, overtaking_state(commitment_since=9.0))
    assert young.event is None


def test_overtake_abandons_without_feasible_point():
    ot = Overtaking(PARAMS, WALKABLE)
    o = obs((14, 5.3), [(1, (15, 5)), (2, (15, 5.8)), (3, (15, 4.2))])
    event = ot.control(o, overtaking_state()).event
    assert event == "overtake_abandoned_no_gap"
