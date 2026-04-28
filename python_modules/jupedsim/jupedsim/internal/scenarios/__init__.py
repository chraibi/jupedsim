# SPDX-License-Identifier: LGPL-3.0-or-later
"""Internal scenario helpers for notebooks and parameter studies.

The public surface is intentionally small:

* :class:`ScenarioConfig` -- declarative scenario description, dict-backed.
* :func:`load_scenario` -- validates the config and parses the geometry.
* :func:`init_scenario` -- builds a :class:`jupedsim.Simulation` from a
  loaded scenario.
* :func:`run_scenario` -- executes the simulation and returns a
  :class:`ScenarioResult`.
* :func:`modify_scenario` -- deep-merge overrides into a config.

This module lives under ``jupedsim.internal``; we make no API stability
promises. It is intended for the documentation notebooks and parameter
studies, not for downstream applications.
"""

from .config import ScenarioConfig, modify_scenario
from .loader import LoadedScenario, load_scenario
from .runner import ScenarioResult, init_scenario, run_scenario

__all__ = [
    "LoadedScenario",
    "ScenarioConfig",
    "ScenarioResult",
    "init_scenario",
    "load_scenario",
    "modify_scenario",
    "run_scenario",
]
