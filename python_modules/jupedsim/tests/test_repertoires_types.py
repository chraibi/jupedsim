# SPDX-License-Identifier: LGPL-3.0-or-later

import dataclasses

import pytest
from jupedsim.repertoires import BehaviouralState, Decision


def test_state_is_immutable():
    state = BehaviouralState("x", "y", None, None)
    with pytest.raises(dataclasses.FrozenInstanceError):
        state.anchor = 3


def test_state_reserves_group_and_unit_ids():
    state = BehaviouralState("x", "y", None, None)
    assert state.group_id is None
    assert state.unit_id == -1


def test_decision_carries_one_repertoire_per_member():
    dec = Decision(unit_id=4, repertoires=("a", "b"))
    assert dec.repertoires == ("a", "b")
    assert dec.unit_id == 4
