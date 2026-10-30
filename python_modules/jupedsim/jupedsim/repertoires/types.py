# SPDX-License-Identifier: LGPL-3.0-or-later

import dataclasses
import enum
import typing
from collections.abc import Sequence


class Capability(enum.Enum):
    """Agent controls a repertoire may require from the simulation model."""

    TARGET_CONTROL = enum.auto()
    DESIRED_SPEED = enum.auto()


@dataclasses.dataclass(frozen=True)
class PerceivedAgent:
    """Another agent as perceived by the observing agent.

    Attributes:
        id: Agent id.
        position: Perceived position.
        progress: Perceived progress along the route.
    """

    id: int
    position: tuple[float, float]
    progress: float


@dataclasses.dataclass(frozen=True)
class Memory:
    """What an agent remembers about its own past.

    Attributes:
        progress_history: ``(time, progress)`` pairs.
        passes: ``(time, passer_id)`` pairs.
        anchor_lost: True if the anchor agent was lost from perception.
    """

    progress_history: tuple[tuple[float, float], ...]
    passes: tuple[tuple[float, int], ...]
    anchor_lost: bool


@dataclasses.dataclass(frozen=True)
class Observation:
    """An agent's own perception and memory at one time."""

    agent_id: int
    time: float
    position: tuple[float, float]
    progress: float
    perceived: tuple[PerceivedAgent, ...]
    memory: Memory


@dataclasses.dataclass(frozen=True)
class BehaviouralState:
    """Behavioural state of one agent.

    Attributes:
        repertoire: Name of the active repertoire.
        execution: Name of the active execution mode.
        anchor: Id of the agent being followed or passed, if any.
        target: Current target position, if any.
        phase: Phase counter within the active repertoire.
        commitment_since: Time at which the repertoire was entered.
        group_id: Group the agent belongs to, if any.
        unit_id: Decision unit the agent belongs to.
    """

    repertoire: str
    execution: str
    anchor: int | None
    target: tuple[float, float] | None
    phase: int = 0
    commitment_since: float = 0.0
    group_id: int | None = None
    unit_id: int = -1


@dataclasses.dataclass(frozen=True)
class DecisionRequest:
    """One decision unit's request to a selector, from one snapshot.

    Attributes:
        unit_id: Decision unit id.
        members: Agent ids of the unit's members.
        snapshot_time: Time of the snapshot.
        decision_index: Index of this decision for the unit.
        observations: One observation per member, in member order.
        states: One state per member, in member order.
        events: Events that triggered the decision.
    """

    unit_id: int
    members: tuple[int, ...]
    snapshot_time: float
    decision_index: int
    observations: tuple[Observation, ...]
    states: tuple[BehaviouralState, ...]
    events: tuple[str, ...]


@dataclasses.dataclass(frozen=True)
class Decision:
    """A selector's answer: one repertoire name per member, in order."""

    unit_id: int
    repertoires: tuple[str, ...]


@dataclasses.dataclass(frozen=True)
class Command:
    """A repertoire's output for one agent.

    Attributes:
        target: Target position to set, or None to leave unchanged.
        desired_speed: Desired speed to set, or None to leave unchanged.
        execution: Name of the execution mode.
        state: State after this control step.
        event: Event emitted by the repertoire, if any.
        handoff: True if the repertoire hands control back for a new
            decision.
    """

    target: tuple[float, float] | None
    desired_speed: float | None
    execution: str
    state: BehaviouralState
    event: str | None = None
    handoff: bool = False


class Repertoire(typing.Protocol):
    """A named behaviour that turns observations into commands."""

    name: str
    requires: frozenset[Capability]

    def enter(
        self, obs: Observation, state: BehaviouralState
    ) -> BehaviouralState:
        """Return the state on entering this repertoire."""
        ...

    def control(self, obs: Observation, state: BehaviouralState) -> Command:
        """Return the command for the current step."""
        ...


class Selector(typing.Protocol):
    """Chooses repertoires for decision units."""

    def decide(self, requests: Sequence[DecisionRequest]) -> Sequence[Decision]:
        """Return one decision per request."""
        ...


class SelectorError(RuntimeError):
    """Raised when a selector fails to produce valid decisions."""
