# SPDX-License-Identifier: LGPL-3.0-or-later

import pytest
import shapely
from jupedsim.repertoires import (
    BehaviouralState,
    DecisionRequest,
    Memory,
    Observation,
    PerceivedAgent,
)
from jupedsim_examples.repertoires import (
    EntranceParameters,
    EntranceRuleSelector,
    RuleParameters,
    progress,
)

ENTRANCE = EntranceParameters()
WALKABLE = shapely.Polygon([(0, 0), (20, 0), (20, 10), (0, 10)])
STALLED = tuple((float(t), -10.0) for t in range(5, 11))
MOVING = tuple((float(t), -10.0 + t) for t in range(5, 11))


def selector():
    return EntranceRuleSelector(RuleParameters(), ENTRANCE, WALKABLE)


def request(
    unit_id=0,
    repertoire="queueing",
    history=STALLED,
    passes=(),
    anchor=1,
    others=((1, (11.0, 5.0)),),
    events=(),
    pos=(10.0, 5.0),
):
    perceived = tuple(
        PerceivedAgent(i, p, progress(ENTRANCE)(p)) for i, p in others
    )
    obs = Observation(
        agent_id=unit_id,
        time=10.0,
        position=pos,
        progress=progress(ENTRANCE)(pos),
        perceived=perceived,
        memory=Memory(history, passes, False),
    )
    state = BehaviouralState(
        repertoire=repertoire,
        execution="standing",
        anchor=anchor,
        target=None,
        unit_id=unit_id,
    )
    return DecisionRequest(
        unit_id=unit_id,
        members=(unit_id,),
        snapshot_time=10.0,
        decision_index=0,
        observations=(obs,),
        states=(state,),
        events=events,
    )


BLOCKERS = ((1, (11.0, 5.0)), (2, (11.0, 5.8)), (3, (11.0, 4.2)))
RECENT_PASSES = ((7.0, 5), (9.0, 6))


def decide(req):
    return selector().decide([req])[0].repertoires


def reasons(req):
    return selector().decide([req])[0].reasons


def test_stalled_with_gap_overtakes():
    assert decide(request()) == ("overtaking",)


def test_stalled_without_gap_keeps_queueing():
    req = request(others=BLOCKERS)
    assert decide(req) == ("queueing",)
    assert reasons(req) == ("overtake_withheld_no_gap",)


def test_stalled_without_anchor_keeps_queueing():
    assert decide(request(anchor=None)) == ("queueing",)


def test_moving_agent_keeps_queueing():
    assert decide(request(history=MOVING)) == ("queueing",)


def test_two_recent_passes_with_gap_trigger_overtaking():
    req = request(history=MOVING, passes=RECENT_PASSES)
    assert decide(req) == ("overtaking",)
    assert reasons(req) == ()


def test_two_recent_passes_without_gap_keep_queueing():
    req = request(history=MOVING, passes=RECENT_PASSES, others=BLOCKERS)
    assert decide(req) == ("queueing",)
    assert reasons(req) == ("overtake_withheld_no_gap",)


def test_two_recent_passes_without_anchor_keep_queueing():
    req = request(history=MOVING, passes=RECENT_PASSES, anchor=None)
    assert decide(req) == ("queueing",)
    assert reasons(req) == ("overtake_withheld_no_gap",)


def test_gap_opening_while_passes_remembered_enters_overtaking():
    blocked = request(history=MOVING, passes=RECENT_PASSES, others=BLOCKERS)
    opened = request(history=MOVING, passes=RECENT_PASSES)
    assert decide(blocked) == ("queueing",)
    assert decide(opened) == ("overtaking",)


def test_unmotivated_agent_gives_no_reason():
    assert reasons(request(history=MOVING)) == ()


def test_old_passes_do_not_count():
    passes = ((3.0, 5), (9.0, 6))
    assert decide(request(history=MOVING, passes=passes)) == ("queueing",)


def test_completed_overtake_returns_to_queueing():
    req = request(repertoire="overtaking", events=("overtake_completed",))
    assert decide(req) == ("queueing",)


@pytest.mark.parametrize(
    "event",
    [
        "overtake_abandoned_no_gap",
        "overtake_abandoned_target_lost",
        "overtake_abandoned_stalled",
    ],
)
def test_abandoned_overtake_returns_to_queueing(event):
    req = request(repertoire="overtaking", events=(event,))
    assert decide(req) == ("queueing",)


def test_overtaking_continues_without_event():
    assert decide(request(repertoire="overtaking")) == ("overtaking",)


def test_other_repertoire_is_kept():
    assert decide(request(repertoire="other")) == ("other",)


def test_decisions_independent_of_request_order():
    reqs = [
        request(unit_id=0),
        request(unit_id=1, history=MOVING),
        request(unit_id=2, repertoire="overtaking"),
    ]
    forward = {d.unit_id: d for d in selector().decide(reqs)}
    backward = selector().decide(reqs[::-1])
    assert [d.unit_id for d in backward] == [2, 1, 0]
    assert all(forward[d.unit_id] == d for d in backward)
