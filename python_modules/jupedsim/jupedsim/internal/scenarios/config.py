# SPDX-License-Identifier: LGPL-3.0-or-later
"""Dict-backed scenario configuration with typed accessors."""

from __future__ import annotations

import copy
import json
import pathlib
from typing import Any, Iterable, Mapping

_REQUIRED_TOP_LEVEL_KEYS = ("geometry", "model")


class ScenarioConfig:
    """Declarative scenario description.

    Dict-backed: the underlying ``raw`` mapping is the single source of
    truth and is what gets serialized to JSON. Typed properties below
    are read-only views; mutation goes through :func:`modify_scenario`.

    Notebooks usually construct a config inline::

        config = ScenarioConfig({
            "seed": 0,
            "geometry": "POLYGON ((...))",
            "model": {"type": "CollisionFreeSpeedModelV2"},
            "stages": {"exit": {"type": "exit", "polygon": "POLYGON ((...))"}},
            "journeys": [{"name": "main", "stages": ["exit"]}],
            "agents": [{"journey": "main", "stage": "exit",
                        "positions": [(1.0, 2.0)]}],
        })

    Round-tripping to JSON is supported via :meth:`from_json` and
    :meth:`to_json`; the on-disk format mirrors the dict layout.
    """

    def __init__(self, raw: Mapping[str, Any] | None = None) -> None:
        data: dict[str, Any] = dict(raw) if raw is not None else {}
        for key in _REQUIRED_TOP_LEVEL_KEYS:
            if key not in data:
                raise ValueError(
                    f"ScenarioConfig is missing required key '{key}'. "
                    f"Required keys: {sorted(_REQUIRED_TOP_LEVEL_KEYS)}."
                )
        model = data["model"]
        if not isinstance(model, Mapping):
            raise ValueError(
                "ScenarioConfig key 'model' must be a mapping containing "
                "required key 'type'."
            )
        if "type" not in model:
            raise ValueError(
                "ScenarioConfig is missing required key 'model.type'."
            )
        if not isinstance(model["type"], str):
            raise ValueError(
                "ScenarioConfig key 'model.type' must be a string."
            )
        self._raw: dict[str, Any] = data

    # ----- IO ----------------------------------------------------------------

    @classmethod
    def from_json(cls, path: str | pathlib.Path) -> "ScenarioConfig":
        """Load a config from a JSON file."""
        text = pathlib.Path(path).read_text(encoding="utf-8")
        return cls(json.loads(text))

    def to_json(self, path: str | pathlib.Path, *, indent: int = 2) -> None:
        """Serialize the config to a JSON file."""
        pathlib.Path(path).write_text(
            json.dumps(self._raw, indent=indent, default=str),
            encoding="utf-8",
        )

    # ----- accessors ---------------------------------------------------------

    @property
    def raw(self) -> dict[str, Any]:
        """Underlying mapping. Avoid mutating; use ``modify_scenario`` instead."""
        return self._raw

    @property
    def seed(self) -> int:
        return int(self._raw.get("seed", 0))

    @property
    def simulation(self) -> Mapping[str, Any]:
        return self._raw.get("simulation", {})

    @property
    def dt(self) -> float:
        return float(self.simulation.get("dt", 0.05))

    @property
    def max_time(self) -> float:
        return float(self.simulation.get("max_time", 300.0))

    @property
    def every_nth_frame(self) -> int:
        return int(self.simulation.get("every_nth_frame", 5))

    @property
    def geometry_wkt(self) -> str:
        return str(self._raw["geometry"])

    @property
    def model_type(self) -> str:
        return str(self._raw["model"]["type"])

    @property
    def model_params(self) -> dict[str, Any]:
        params = dict(self._raw["model"])
        params.pop("type", None)
        return params

    @property
    def stages(self) -> Mapping[str, Mapping[str, Any]]:
        return self._raw.get("stages", {})

    @property
    def journeys(self) -> list[Mapping[str, Any]]:
        return list(self._raw.get("journeys", []))

    @property
    def agents(self) -> list[Mapping[str, Any]]:
        return list(self._raw.get("agents", []))

    @property
    def extra(self) -> Mapping[str, Any]:
        return self._raw.get("extra", {})

    # ----- dunders -----------------------------------------------------------

    def __repr__(self) -> str:
        return (
            f"ScenarioConfig(model={self.model_type!r}, "
            f"stages={list(self.stages)!r}, "
            f"journeys={[j.get('name') for j in self.journeys]!r}, "
            f"agent_groups={len(self.agents)})"
        )


# ----- modify_scenario --------------------------------------------------------


def _deep_merge(base: Any, override: Any) -> Any:
    """Recursive dict merge: override wins, lists are replaced wholesale."""
    if isinstance(base, dict) and isinstance(override, dict):
        out = dict(base)
        for k, v in override.items():
            out[k] = _deep_merge(out.get(k), v) if k in out else v
        return out
    return override


def modify_scenario(
    config: ScenarioConfig,
    *,
    overrides: Mapping[str, Any] | None = None,
    **kwargs: Any,
) -> ScenarioConfig:
    """Return a new :class:`ScenarioConfig` with deep-merged overrides.

    ``overrides`` may be supplied as a single mapping (suitable for
    deeply nested keys), as keyword arguments (top-level only), or both.
    Lists in the original config are replaced wholesale; nested dicts
    are merged.

    Example::

        sweep = [
            modify_scenario(
                base, overrides={"model": {"strength_neighbor_repulsion": s}}
            )
            for s in (4.0, 6.0, 8.0)
        ]
    """
    merged: dict[str, Any] = copy.deepcopy(config.raw)
    if overrides:
        merged = _deep_merge(merged, dict(overrides))
    if kwargs:
        merged = _deep_merge(merged, kwargs)
    return ScenarioConfig(merged)


def _validate_stage_refs(
    journeys: Iterable[Mapping[str, Any]], stage_names: Iterable[str]
) -> None:
    """Ensure every stage referenced in a journey is defined in ``stages``."""
    known = set(stage_names)
    for journey in journeys:
        name = journey.get("name", "<unnamed>")
        for stage in journey.get("stages", []):
            if stage not in known:
                raise ValueError(
                    f"Journey '{name}' references undefined stage '{stage}'. "
                    f"Defined stages: {sorted(known)}."
                )
