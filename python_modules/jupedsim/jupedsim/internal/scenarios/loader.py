# SPDX-License-Identifier: LGPL-3.0-or-later
"""Validate a :class:`ScenarioConfig` and parse its WKT geometry."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

import shapely
from shapely.geometry.base import BaseGeometry

from .config import ScenarioConfig, _validate_stage_refs

_KNOWN_STAGE_TYPES = (
    "waypoint",
    "exit",
    "waiting_set",
    "notifiable_queue",
    "direct_steering",
)


@dataclass
class LoadedScenario:
    """A :class:`ScenarioConfig` with its geometry parsed and validated."""

    config: ScenarioConfig
    walkable_area: BaseGeometry
    stage_geometries: dict[str, BaseGeometry]
    distribution_polygons: dict[int, BaseGeometry]


def _parse_wkt(wkt_str: str, what: str) -> BaseGeometry:
    geom = shapely.from_wkt(wkt_str)
    if geom.is_empty:
        raise ValueError(f"{what}: WKT parsed to an empty geometry")
    return geom


def _validate_stage_spec(name: str, spec: Mapping[str, Any]) -> None:
    if "type" not in spec:
        raise ValueError(f"Stage '{name}' is missing 'type'")
    stype = spec["type"]
    if stype not in _KNOWN_STAGE_TYPES:
        raise ValueError(
            f"Stage '{name}' has unknown type '{stype}'. "
            f"Known: {list(_KNOWN_STAGE_TYPES)}."
        )
    if stype == "waypoint":
        if "position" not in spec or "distance" not in spec:
            raise ValueError(
                f"Waypoint stage '{name}' needs 'position' and 'distance'"
            )
    elif stype == "exit":
        if "polygon" not in spec:
            raise ValueError(f"Exit stage '{name}' needs 'polygon' (WKT)")
    elif stype in ("waiting_set", "notifiable_queue"):
        if "positions" not in spec:
            raise ValueError(
                f"{stype} stage '{name}' needs 'positions' (list of (x, y))"
            )
    # direct_steering carries no geometry payload.


def load_scenario(config: ScenarioConfig) -> LoadedScenario:
    """Parse and validate ``config`` without building a simulation.

    Raises :class:`ValueError` on missing keys, unknown stage types, or
    journeys referencing stages that aren't defined.
    """
    walkable = _parse_wkt(config.geometry_wkt, "walkable area")

    stage_geoms: dict[str, BaseGeometry] = {}
    for name, spec in config.stages.items():
        _validate_stage_spec(name, spec)
        if spec["type"] == "exit":
            stage_geoms[name] = _parse_wkt(spec["polygon"], f"stage '{name}'")

    _validate_stage_refs(config.journeys, list(config.stages.keys()))
    _validate_direct_steering_journeys(config.journeys, config.stages)

    journey_names = {
        j.get("name", f"journey_{i}") for i, j in enumerate(config.journeys)
    }
    stage_names = set(config.stages.keys())

    distribution_polygons: dict[int, BaseGeometry] = {}
    for idx, group in enumerate(config.agents):
        if "journey" not in group or "stage" not in group:
            raise ValueError(
                f"Agent group #{idx} is missing 'journey' or 'stage'"
            )
        if group["journey"] not in journey_names:
            raise ValueError(
                f"Agent group #{idx} references undefined journey "
                f"'{group['journey']}'. Defined journeys: "
                f"{sorted(journey_names)}."
            )
        if group["stage"] not in stage_names:
            raise ValueError(
                f"Agent group #{idx} references undefined stage "
                f"'{group['stage']}'. Defined stages: "
                f"{sorted(stage_names)}."
            )
        if "distribution" in group and "positions" in group:
            raise ValueError(
                f"Agent group #{idx} declares both 'distribution' and "
                f"'positions'; pick one."
            )
        if "distribution" in group:
            dist = group["distribution"]
            for required in ("polygon", "number"):
                if required not in dist:
                    raise ValueError(
                        f"Agent group #{idx}: distribution is missing "
                        f"'{required}'"
                    )
            distribution_polygons[idx] = _parse_wkt(
                dist["polygon"], f"agent group #{idx} distribution"
            )

    return LoadedScenario(
        config=config,
        walkable_area=walkable,
        stage_geometries=stage_geoms,
        distribution_polygons=distribution_polygons,
    )


def _validate_direct_steering_journeys(
    journeys: list[Mapping[str, Any]],
    stages: Mapping[str, Mapping[str, Any]],
) -> None:
    """``direct_steering`` stages may only appear alone in a journey."""
    for journey in journeys:
        names = list(journey.get("stages", []))
        is_direct = [
            stages.get(n, {}).get("type") == "direct_steering" for n in names
        ]
        if any(is_direct) and len(names) > 1:
            raise ValueError(
                f"Journey '{journey.get('name', '<unnamed>')}' mixes a "
                f"'direct_steering' stage with other stages; a "
                f"direct_steering stage must be the only stage in its "
                f"journey."
            )
