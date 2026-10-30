# SPDX-License-Identifier: LGPL-3.0-or-later

import dataclasses
import json
import pathlib
from collections.abc import Mapping
from typing import Any

from jupedsim.repertoires.types import (
    Decision,
    DecisionRequest,
    Observation,
)


class RunLog:
    """JSON Lines log of one repertoire run.

    The first line is a ``metadata`` record. Every further line is one
    record with a ``type`` key. Keys are sorted so that identical runs
    produce identical files.
    """

    def __init__(self, path: pathlib.Path, metadata: Mapping[str, Any]):
        """Open the log and write the metadata record.

        Args:
            path: File to write.
            metadata: Selector name and config, backend/model version,
                seed and parameters of the run.
        """
        self._file = open(path, "w", encoding="utf-8")
        self._last_execution: dict[int, tuple[str, str]] = {}
        self._write({"type": "metadata", **metadata})

    def _write(self, record: Mapping[str, Any]) -> None:
        self._file.write(json.dumps(record, sort_keys=True) + "\n")

    def observation(self, obs: Observation) -> None:
        """Log one agent observation."""
        self._write({"type": "observation", **dataclasses.asdict(obs)})

    def request(self, req: DecisionRequest) -> None:
        """Log one decision request."""
        self._write({"type": "request", **dataclasses.asdict(req)})

    def response(
        self, dec: Decision, decision_index: int, snapshot_time: float
    ) -> None:
        """Log one selector decision with its request coordinates."""
        self._write(
            {
                "type": "response",
                **dataclasses.asdict(dec),
                "decision_index": decision_index,
                "snapshot_time": snapshot_time,
            }
        )

    def transition(
        self,
        time: float,
        agent_id: int,
        unit_id: int,
        group_id: int | None,
        before: str,
        after: str,
        cause: str,
    ) -> None:
        """Log a repertoire transition of one agent."""
        self._write(
            {
                "type": "transition",
                "time": time,
                "agent_id": agent_id,
                "unit_id": unit_id,
                "group_id": group_id,
                "before": before,
                "after": after,
                "cause": cause,
            }
        )

    def execution(
        self, time: float, agent_id: int, repertoire: str, execution: str
    ) -> None:
        """Log an execution mode; written only when it changes."""
        current = (repertoire, execution)
        if self._last_execution.get(agent_id) == current:
            return
        self._last_execution[agent_id] = current
        self._write(
            {
                "type": "execution",
                "time": time,
                "agent_id": agent_id,
                "repertoire": repertoire,
                "execution": execution,
            }
        )

    def diagnostic(
        self, time: float, kind: str, detail: Mapping[str, Any]
    ) -> None:
        """Log a global diagnostic that is never passed to agents."""
        self._write(
            {
                "type": "diagnostic",
                "time": time,
                "kind": kind,
                "detail": dict(detail),
            }
        )

    def close(self) -> None:
        """Flush and close the log."""
        if self._file.closed:
            return
        self._file.flush()
        self._file.close()

    def __enter__(self) -> "RunLog":
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()


def load_records(path: pathlib.Path) -> list[dict]:
    """Read all records of a log file."""
    with open(path, encoding="utf-8") as file:
        return [json.loads(line) for line in file if line.strip()]


def transitions_by_agent(path: pathlib.Path) -> dict[int, list[dict]]:
    """Group transition records by agent id, sorted by time."""
    result: dict[int, list[dict]] = {}
    for record in load_records(path):
        if record["type"] == "transition":
            result.setdefault(record["agent_id"], []).append(record)
    for records in result.values():
        records.sort(key=lambda r: r["time"])
    return result
