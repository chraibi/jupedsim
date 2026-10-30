# SPDX-License-Identifier: LGPL-3.0-or-later

import dataclasses
import types
from collections.abc import Mapping, Sequence

import shapely

from jupedsim.repertoires.diagnostics import anchor_cycles, anchor_structure
from jupedsim.repertoires.log import RunLog
from jupedsim.repertoires.perception import (
    MemoryTracker,
    Perception,
    passes_since,
)
from jupedsim.repertoires.selectors import validate
from jupedsim.repertoires.types import (
    BehaviouralState,
    Capability,
    Command,
    Decision,
    DecisionRequest,
    Memory,
    Observation,
    Repertoire,
    Selector,
    SelectorError,
)

_EMPTY_MEMORY = Memory(progress_history=(), passes=(), anchor_lost=False)

# Margin for comparing simulation time with the periodic schedule.
_TIME_TOLERANCE = 1e-9


def model_capabilities(state) -> frozenset[Capability]:
    """Capabilities offered by an agent's model state.

    Args:
        state: Built-in model state handle of one agent, as returned by
            ``sim.agent(agent_id).state``.

    Returns:
        ``TARGET_CONTROL`` always; ``DESIRED_SPEED`` if the state has a
        ``desired_speed``.
    """
    if hasattr(state, "desired_speed"):
        return frozenset({Capability.TARGET_CONTROL, Capability.DESIRED_SPEED})
    return frozenset({Capability.TARGET_CONTROL})


class RepertoireController:
    """Drives agents of a steering journey through behavioural repertoires.

    Each call of :meth:`step` observes all controlled agents at one
    snapshot, asks the selector for decisions of all due units at once,
    applies them together, lets every agent's repertoire control it and
    then advances the simulation by ``control_interval`` iterations.
    Agents and units are processed in ascending id order, so results do
    not depend on the order in which agents were added.

    Each unit gets a run-relative index, from 0, when it first enters a
    repertoire; units entering in the same step are indexed in ascending
    id order. The index sets the unit's phase offset
    ``(index % 10) / 10 * decision_period`` and is passed to the selector
    as ``DecisionRequest.unit_index``, so results do not depend on the
    global agent ids.

    Args:
        sim: The simulation.
        repertoires: Repertoires by name.
        selector: Chooses repertoires for due units.
        perception: Builds observations.
        memory: Tracks each agent's memory.
        log: Run log.
        steering_journey: Journey of controlled agents.
        steering_stage: Direct steering stage of that journey.
        exit_journey: Journey an agent is switched to on handoff.
        exit_stage: Stage of the exit journey.
        walkable: Area in which targets are accepted.
        control_interval: Iterations per :meth:`step`.
        decision_period: Seconds between periodic decisions of a unit.
        capabilities: Capabilities offered by the model.

    Raises:
        ValueError: If a repertoire requires a capability not offered.
    """

    def __init__(
        self,
        sim,
        *,
        repertoires: Mapping[str, Repertoire],
        selector: Selector,
        perception: Perception,
        memory: MemoryTracker,
        log: RunLog,
        steering_journey: int,
        steering_stage: int,
        exit_journey: int,
        exit_stage: int,
        walkable: shapely.Polygon,
        control_interval: int,
        decision_period: float,
        capabilities: frozenset[Capability],
    ):
        for name, repertoire in repertoires.items():
            missing = repertoire.requires - capabilities
            if missing:
                raise ValueError(
                    f"repertoire {name!r} requires unavailable "
                    f"capabilities {sorted(c.name for c in missing)}"
                )
        self._sim = sim
        self._repertoires = dict(repertoires)
        self._selector = selector
        self._perception = perception
        self._memory = memory
        self._log = log
        self._steering_journey = steering_journey
        self._steering_stage = steering_stage
        self._exit_journey = exit_journey
        self._exit_stage = exit_stage
        self._walkable = walkable
        self._control_interval = control_interval
        self._decision_period = decision_period
        self._states: dict[int, BehaviouralState] = {}
        self._memories: dict[int, Memory] = {}
        self._entered: set[int] = set()
        self._pending: dict[int, str | None] = {}
        self._next_due: dict[int, float] = {}
        self._decision_index: dict[int, int] = {}
        self._unit_index: dict[int, int] = {}
        self._next_unit_index = 0

    @property
    def states(self) -> Mapping[int, BehaviouralState]:
        """Behavioural state of every controlled agent (read-only)."""
        return types.MappingProxyType(self._states)

    def add(self, agent_id: int, repertoire: str) -> None:
        """Put an agent under control of the layer.

        The agent forms its own decision unit with ``unit_id = agent_id``.
        Its run-relative unit index is assigned at the next :meth:`step`.

        Args:
            agent_id: Agent already following the steering journey.
            repertoire: Name of the initial repertoire.

        Raises:
            ValueError: If the agent is already controlled, the
                repertoire is unknown or the agent is not in the steering
                journey.
        """
        if agent_id in self._states:
            raise ValueError(f"agent {agent_id} is already controlled")
        if repertoire not in self._repertoires:
            raise ValueError(f"unknown repertoire {repertoire!r}")
        agent = self._sim.agent(agent_id)
        if (agent.journey_id, agent.stage_id) != (
            self._steering_journey,
            self._steering_stage,
        ):
            raise ValueError(f"agent {agent_id} is not at the steering stage")
        self._states[agent_id] = BehaviouralState(
            repertoire=repertoire,
            execution="",
            anchor=None,
            target=None,
            unit_id=agent_id,
        )
        self._memories[agent_id] = _EMPTY_MEMORY
        self._pending[agent_id] = None
        self._decision_index[agent_id] = 0

    def step(self) -> None:
        """Run one control interval.

        Raises:
            SelectorError: If the selector fails or violates its
                contract; the simulation is then not advanced.
        """
        now = self._sim.elapsed_time()
        live = self._drop_departed()
        observations = self._observe(now)
        self._enter_new(observations)
        requests = self._collect_requests(observations, now)
        decisions = self._decide(requests)
        self._apply_decisions(requests, decisions, observations)
        self._control_all(observations, live, now)
        self._log_diagnostics(now)
        self._sim.iterate(self._control_interval)

    def _log_diagnostics(self, now: float) -> None:
        anchors = {i: s.anchor for i, s in sorted(self._states.items())}
        self._log.diagnostic(now, "anchor_structure", anchor_structure(anchors))
        for cycle in anchor_cycles(anchors):
            self._log.diagnostic(now, "anchor_cycle", {"members": list(cycle)})

    def _drop_departed(self) -> set[int]:
        live = {agent.id for agent in self._sim.agents()}
        for agent_id in [i for i in self._states if i not in live]:
            self._release(agent_id)
            self._memory.forget(agent_id)
        for agent_id, state in sorted(self._states.items()):
            if state.anchor is not None and state.anchor not in live:
                self._memory.mark_anchor_lost(agent_id)
        return live

    def _release(self, agent_id: int) -> None:
        for table in (
            self._states,
            self._memories,
            self._pending,
            self._next_due,
            self._decision_index,
            self._unit_index,
        ):
            table.pop(agent_id, None)
        self._entered.discard(agent_id)

    def _observe(self, now: float) -> dict[int, Observation]:
        return {
            agent_id: self._observe_agent(agent_id, now)
            for agent_id in sorted(self._states)
        }

    def _observe_agent(self, agent_id: int, now: float) -> Observation:
        obs = self._perception.observe(
            self._sim,
            agent_id,
            now,
            self._memories[agent_id],
            tracked=self._tracked(agent_id),
        )
        memory = self._memory.update(agent_id, now, obs.progress, obs.perceived)
        self._memories[agent_id] = memory
        obs = dataclasses.replace(obs, memory=memory)
        self._log.observation(obs)
        return obs

    def _tracked(self, agent_id: int) -> int | None:
        state = self._states[agent_id]
        repertoire = self._repertoires[state.repertoire]
        if not getattr(repertoire, "tracks_anchor", False):
            return None
        return state.anchor

    def _enter_new(self, observations: Mapping[int, Observation]) -> None:
        for agent_id in sorted(set(self._states) - self._entered):
            state = self._states[agent_id]
            repertoire = self._repertoires[state.repertoire]
            self._states[agent_id] = repertoire.enter(
                observations[agent_id], state
            )
            self._entered.add(agent_id)
            self._assign_index(agent_id)

    def _assign_index(self, unit_id: int) -> None:
        index = self._next_unit_index
        self._next_unit_index += 1
        self._unit_index[unit_id] = index
        self._next_due[unit_id] = (index % 10) / 10 * self._decision_period

    def _collect_requests(
        self, observations: Mapping[int, Observation], now: float
    ) -> list[DecisionRequest]:
        requests = []
        for unit_id in sorted(self._states):
            request = self._request(unit_id, observations[unit_id], now)
            if request is not None:
                requests.append(request)
        return requests

    def _request(
        self, unit_id: int, obs: Observation, now: float
    ) -> DecisionRequest | None:
        periodic = self._periodic_due(unit_id, now)
        events = self._events(unit_id, obs, now)
        if not periodic and not events:
            return None
        request = DecisionRequest(
            unit_id=unit_id,
            members=(unit_id,),
            snapshot_time=now,
            decision_index=self._decision_index[unit_id],
            observations=(obs,),
            states=(self._states[unit_id],),
            events=events,
            unit_index=self._unit_index[unit_id],
        )
        self._decision_index[unit_id] += 1
        self._log.request(request)
        return request

    def _periodic_due(self, unit_id: int, now: float) -> bool:
        if now + _TIME_TOLERANCE < self._next_due[unit_id]:
            return False
        while self._next_due[unit_id] <= now + _TIME_TOLERANCE:
            self._next_due[unit_id] += self._decision_period
        return True

    def _events(
        self, unit_id: int, obs: Observation, now: float
    ) -> tuple[str, ...]:
        events = set()
        if self._pending[unit_id] is not None:
            events.add(self._pending[unit_id])
        if passes_since(obs.memory, now) > 0:
            events.add("pass_observed")
        if obs.memory.anchor_lost:
            events.add("anchor_lost")
        return tuple(sorted(events))

    def _decide(
        self, requests: Sequence[DecisionRequest]
    ) -> dict[int, Decision]:
        if not requests:
            return {}
        try:
            decisions = list(self._selector.decide(requests))
        except SelectorError:
            raise
        except Exception as exc:
            raise SelectorError(f"selector failed: {exc}") from exc
        validate(requests, decisions)
        self._check_names(decisions)
        by_unit = {dec.unit_id: dec for dec in decisions}
        for req in requests:
            self._log.response(
                by_unit[req.unit_id],
                req.decision_index,
                req.snapshot_time,
                unit_index=req.unit_index,
            )
        return by_unit

    def _check_names(self, decisions: Sequence[Decision]) -> None:
        for dec in decisions:
            unknown = set(dec.repertoires) - set(self._repertoires)
            if unknown:
                raise SelectorError(
                    f"unit {dec.unit_id}: unknown repertoires {sorted(unknown)}"
                )

    def _apply_decisions(
        self,
        requests: Sequence[DecisionRequest],
        decisions: Mapping[int, Decision],
        observations: Mapping[int, Observation],
    ) -> None:
        for req in requests:
            cause = ",".join(req.events) or "periodic"
            names = decisions[req.unit_id].repertoires
            for agent_id, name in zip(req.members, names):
                self._switch(agent_id, name, observations[agent_id], cause)

    def _switch(
        self, agent_id: int, name: str, obs: Observation, cause: str
    ) -> None:
        state = self._states[agent_id]
        if name == state.repertoire:
            return
        self._states[agent_id] = self._repertoires[name].enter(
            obs, dataclasses.replace(state, repertoire=name)
        )
        self._log.transition(
            obs.time,
            agent_id,
            state.unit_id,
            state.group_id,
            state.repertoire,
            name,
            cause,
        )

    def _control_all(
        self,
        observations: Mapping[int, Observation],
        live: set[int],
        now: float,
    ) -> None:
        for agent_id in sorted(self._states):
            self._control_agent(agent_id, observations[agent_id], live, now)

    def _control_agent(
        self, agent_id: int, obs: Observation, live: set[int], now: float
    ) -> None:
        state = self._states[agent_id]
        command = self._repertoires[state.repertoire].control(obs, state)
        agent = self._sim.agent(agent_id)
        state = self._apply_target(agent, command, obs)
        if command.desired_speed is not None:
            agent.state.desired_speed = command.desired_speed
        self._states[agent_id] = state
        self._pending[agent_id] = command.event
        if obs.memory.anchor_lost and (
            state.anchor is None or state.anchor in live
        ):
            self._memory.clear_anchor_lost(agent_id)
        self._log.execution(now, agent_id, state.repertoire, state.execution)
        if command.handoff:
            self._log.handoff(now, agent_id, state.unit_id, state.repertoire)
            self._sim.switch_agent_journey(
                agent_id, self._exit_journey, self._exit_stage
            )
            self._release(agent_id)

    def _apply_target(
        self, agent, command: Command, obs: Observation
    ) -> BehaviouralState:
        state = dataclasses.replace(command.state, execution=command.execution)
        if command.target is None:
            return state
        if self._walkable.covers(shapely.Point(command.target)):
            agent.final_target = command.target
            return state
        agent.final_target = obs.position
        return dataclasses.replace(
            state, execution="standing", target=obs.position
        )
