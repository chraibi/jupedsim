# SPDX-License-Identifier: LGPL-3.0-or-later
"""Round-trip tests for the internal scenario helpers."""

import json
import pathlib

import pytest
from jupedsim import SqliteTrajectoryWriter as jps_sqlite_writer  # noqa: N813
from jupedsim.internal.scenarios import (
    ScenarioConfig,
    init_scenario,
    load_scenario,
    modify_scenario,
    run_scenario,
)

ROOM_WKT = "POLYGON ((0 0, 10 0, 10 10, 0 10, 0 0))"
EXIT_WKT = "POLYGON ((9 0, 10 0, 10 10, 9 10, 9 0))"
SPAWN_WKT = "POLYGON ((1 1, 4 1, 4 9, 1 9, 1 1))"


def _base_config() -> dict:
    return {
        "seed": 0,
        "simulation": {"dt": 0.05, "max_time": 30.0, "every_nth_frame": 2},
        "geometry": ROOM_WKT,
        "model": {"type": "CollisionFreeSpeedModelV2"},
        "stages": {
            "exit": {"type": "exit", "polygon": EXIT_WKT},
        },
        "journeys": [{"name": "main", "stages": ["exit"]}],
        "agents": [
            {
                "journey": "main",
                "stage": "exit",
                "v0": 1.2,
                "radius": 0.15,
                "positions": [(2.0, 5.0), (3.0, 5.0)],
            },
        ],
    }


# ----- ScenarioConfig --------------------------------------------------------


def test_config_requires_geometry_and_model():
    with pytest.raises(ValueError, match="geometry"):
        ScenarioConfig({"model": {"type": "CollisionFreeSpeedModelV2"}})
    with pytest.raises(ValueError, match="model"):
        ScenarioConfig({"geometry": ROOM_WKT})


def test_config_typed_accessors():
    config = ScenarioConfig(_base_config())
    assert config.seed == 0
    assert config.dt == 0.05
    assert config.max_time == 30.0
    assert config.every_nth_frame == 2
    assert config.model_type == "CollisionFreeSpeedModelV2"
    assert "exit" in config.stages
    assert config.journeys[0]["name"] == "main"
    assert len(config.agents) == 1


def test_config_json_roundtrip(tmp_path: pathlib.Path):
    original = ScenarioConfig(_base_config())
    path = tmp_path / "scenario.json"
    original.to_json(path)
    restored = ScenarioConfig.from_json(path)
    # Tuples are flattened to lists by JSON; compare via a JSON-normalized
    # form so the round-trip is byte-identical at the JSON level.
    assert json.dumps(restored.raw, sort_keys=True, default=str) == json.dumps(
        original.raw, sort_keys=True, default=str
    )


def test_modify_scenario_deep_merges():
    base = ScenarioConfig(_base_config())
    derived = modify_scenario(
        base, overrides={"model": {"strength_neighbor_repulsion": 12.0}}
    )
    assert derived.model_params["strength_neighbor_repulsion"] == 12.0
    assert derived.model_type == "CollisionFreeSpeedModelV2"
    # base must be unchanged
    assert "strength_neighbor_repulsion" not in base.model_params


def test_modify_scenario_kwargs():
    base = ScenarioConfig(_base_config())
    derived = modify_scenario(base, seed=42)
    assert derived.seed == 42
    assert base.seed == 0


# ----- load_scenario ---------------------------------------------------------


def test_load_validates_unknown_stage_type():
    raw = _base_config()
    raw["stages"]["bogus"] = {"type": "made_up"}
    raw["journeys"][0]["stages"].append("bogus")
    with pytest.raises(ValueError, match="unknown type"):
        load_scenario(ScenarioConfig(raw))


def test_load_validates_journey_references_undefined_stage():
    raw = _base_config()
    raw["journeys"][0]["stages"].append("missing")
    with pytest.raises(ValueError, match="undefined stage"):
        load_scenario(ScenarioConfig(raw))


def test_config_requires_model_type():
    with pytest.raises(ValueError, match="model.type"):
        ScenarioConfig({"geometry": ROOM_WKT, "model": {}})


def test_load_rejects_undefined_journey_in_agent_group():
    raw = _base_config()
    raw["agents"][0]["journey"] = "ghost"
    with pytest.raises(ValueError, match="undefined journey"):
        load_scenario(ScenarioConfig(raw))


def test_load_rejects_undefined_stage_in_agent_group():
    raw = _base_config()
    raw["agents"][0]["stage"] = "ghost"
    with pytest.raises(ValueError, match="undefined stage"):
        load_scenario(ScenarioConfig(raw))


def test_load_rejects_direct_steering_mixed_with_other_stages():
    raw = _base_config()
    raw["stages"]["wp"] = {
        "type": "waypoint",
        "position": (5.0, 5.0),
        "distance": 0.4,
    }
    raw["stages"]["ds"] = {"type": "direct_steering"}
    raw["journeys"][0]["stages"] = ["wp", "ds"]
    with pytest.raises(ValueError, match="direct_steering"):
        load_scenario(ScenarioConfig(raw))


def test_load_rejects_both_distribution_and_positions():
    raw = _base_config()
    raw["agents"][0]["distribution"] = {"polygon": SPAWN_WKT, "number": 5}
    with pytest.raises(ValueError, match="both 'distribution' and 'positions'"):
        load_scenario(ScenarioConfig(raw))


def test_load_parses_geometry():
    loaded = load_scenario(ScenarioConfig(_base_config()))
    assert loaded.walkable_area.bounds == (0.0, 0.0, 10.0, 10.0)
    assert "exit" in loaded.stage_geometries


# ----- init + run ------------------------------------------------------------


def test_run_short_simulation_to_completion(tmp_path: pathlib.Path):
    raw = _base_config()
    raw["simulation"]["max_time"] = 30.0
    config = ScenarioConfig(raw)
    loaded = load_scenario(config)
    sim = init_scenario(loaded)

    assert sim.agent_count() == 2

    out = tmp_path / "traj.sqlite"
    result = run_scenario(sim, max_time=config.max_time, output_file=out)

    assert result.success
    assert result.agents_remaining == 0
    assert result.iterations > 0
    assert result.trajectory_file.exists()
    assert result.trajectory_file.stat().st_size > 0


def test_run_with_distribution_group(tmp_path: pathlib.Path):
    raw = _base_config()
    raw["agents"] = [
        {
            "journey": "main",
            "stage": "exit",
            "v0": 1.2,
            "radius": 0.15,
            "distribution": {
                "polygon": SPAWN_WKT,
                "number": 5,
                "min_distance": 0.4,
            },
        }
    ]
    config = ScenarioConfig(raw)
    sim = init_scenario(load_scenario(config))
    assert sim.agent_count() == 5
    result = run_scenario(
        sim, max_time=10.0, output_file=tmp_path / "traj.sqlite"
    )
    assert result.iterations > 0


def test_run_writes_initial_frame(tmp_path: pathlib.Path):
    """The trajectory file must include the iteration-0 state."""
    import sqlite3

    config = ScenarioConfig(_base_config())
    sim = init_scenario(load_scenario(config))
    out = tmp_path / "traj.sqlite"
    run_scenario(sim, max_time=30.0, output_file=out, every_nth_frame=1)
    with sqlite3.connect(out) as con:
        first_frame = con.execute(
            "SELECT MIN(frame) FROM trajectory_data"
        ).fetchone()[0]
    assert first_frame == 0


def test_run_with_custom_writer_returns_trajectory_path(tmp_path: pathlib.Path):
    """Even with an injected writer the result exposes its output file."""
    config = ScenarioConfig(_base_config())
    sim = init_scenario(load_scenario(config))
    out = tmp_path / "custom.sqlite"
    custom_writer = jps_sqlite_writer(output_file=out, every_nth_frame=2)
    result = run_scenario(sim, max_time=30.0, trajectory_writer=custom_writer)
    assert result.trajectory_file is not None
    assert result.trajectory_file.resolve() == out.resolve()


def test_parameter_sweep_via_modify_scenario(tmp_path: pathlib.Path):
    base = ScenarioConfig(_base_config())
    seeds = [0, 1, 2]
    iter_counts = []
    for seed in seeds:
        cfg = modify_scenario(base, seed=seed)
        sim = init_scenario(load_scenario(cfg))
        result = run_scenario(
            sim,
            max_time=30.0,
            output_file=tmp_path / f"traj_{seed}.sqlite",
        )
        iter_counts.append(result.iterations)
    assert all(c > 0 for c in iter_counts)
