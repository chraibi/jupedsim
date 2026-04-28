# SPDX-License-Identifier: LGPL-3.0-or-later
"""Registries mapping ``model.type`` strings to JuPedSim constructors.

The registry indirection lets scenario JSON describe operational models
by name (e.g. ``"CollisionFreeSpeedModelV2"``), and lets parameter
sweeps swap models by string. Per-model parameter dicts are passed
through largely unchanged; the few naming oddities (e.g. SocialForce
``body_force``/``friction``) are handled per builder.
"""

from __future__ import annotations

from typing import Any, Callable, Mapping

import jupedsim as jps

ModelBuilder = Callable[[Mapping[str, Any]], Any]
AgentParamsBuilder = Callable[..., Any]


def _cfsm(p: Mapping[str, Any]) -> Any:
    return jps.CollisionFreeSpeedModel(
        strength_neighbor_repulsion=p.get("strength_neighbor_repulsion", 8.0),
        range_neighbor_repulsion=p.get("range_neighbor_repulsion", 0.1),
        strength_geometry_repulsion=p.get("strength_geometry_repulsion", 5.0),
        range_geometry_repulsion=p.get("range_geometry_repulsion", 0.02),
    )


def _cfsm_v2(_p: Mapping[str, Any]) -> Any:
    # CFSM v2 takes its parameters per agent, not on the model itself.
    return jps.CollisionFreeSpeedModelV2()


def _avm(p: Mapping[str, Any]) -> Any:
    kwargs: dict[str, Any] = {}
    if "pushout_strength" in p:
        kwargs["pushout_strength"] = p["pushout_strength"]
    if "rng_seed" in p:
        kwargs["rng_seed"] = p["rng_seed"]
    return jps.AnticipationVelocityModel(**kwargs)


def _gcfm(p: Mapping[str, Any]) -> Any:
    return jps.GeneralizedCentrifugalForceModel(
        strength_neighbor_repulsion=p.get("strength_neighbor_repulsion", 0.3),
        strength_geometry_repulsion=p.get("strength_geometry_repulsion", 0.2),
        max_neighbor_interaction_distance=p.get(
            "max_neighbor_interaction_distance", 2.0
        ),
        max_geometry_interaction_distance=p.get(
            "max_geometry_interaction_distance", 2.0
        ),
        max_neighbor_repulsion_force=p.get("max_neighbor_repulsion_force", 9.0),
        max_geometry_repulsion_force=p.get("max_geometry_repulsion_force", 3.0),
    )


def _social_force(p: Mapping[str, Any]) -> Any:
    return jps.SocialForceModel(
        body_force=p.get("body_force", 2000),
        friction=p.get("friction", 0.08),
    )


_MODEL_BUILDERS: dict[str, ModelBuilder] = {
    "CollisionFreeSpeedModel": _cfsm,
    "CollisionFreeSpeedModelV2": _cfsm_v2,
    "AnticipationVelocityModel": _avm,
    "GeneralizedCentrifugalForceModel": _gcfm,
    "SocialForceModel": _social_force,
}


def build_model(model_type: str, params: Mapping[str, Any]) -> Any:
    """Construct an operational model by name."""
    try:
        builder = _MODEL_BUILDERS[model_type]
    except KeyError as exc:
        raise ValueError(
            f"Unknown model type '{model_type}'. "
            f"Known: {sorted(_MODEL_BUILDERS)}"
        ) from exc
    return builder(params)


def _gcfm_agent_params(*, position, journey_id, stage_id, v0, radius, **_):
    return jps.GeneralizedCentrifugalForceModelAgentParameters(
        position=position,
        journey_id=journey_id,
        stage_id=stage_id,
        desired_speed=v0,
        a_v=1.0,
        a_min=radius,
        b_min=radius,
        b_max=radius * 2,
    )


_AGENT_PARAMS_BUILDERS: dict[str, AgentParamsBuilder] = {
    "CollisionFreeSpeedModel": (
        lambda *, position, journey_id, stage_id, v0, radius, **_: (
            jps.CollisionFreeSpeedModelAgentParameters(
                position=position,
                journey_id=journey_id,
                stage_id=stage_id,
                desired_speed=v0,
                radius=radius,
            )
        )
    ),
    "CollisionFreeSpeedModelV2": (
        lambda *, position, journey_id, stage_id, v0, radius, **_: (
            jps.CollisionFreeSpeedModelV2AgentParameters(
                position=position,
                journey_id=journey_id,
                stage_id=stage_id,
                desired_speed=v0,
                radius=radius,
            )
        )
    ),
    "AnticipationVelocityModel": (
        lambda *, position, journey_id, stage_id, v0, radius, **_: (
            jps.AnticipationVelocityModelAgentParameters(
                position=position,
                journey_id=journey_id,
                stage_id=stage_id,
                desired_speed=v0,
                radius=radius,
            )
        )
    ),
    "GeneralizedCentrifugalForceModel": _gcfm_agent_params,
    "SocialForceModel": (
        lambda *, position, journey_id, stage_id, v0, radius, **_: (
            jps.SocialForceModelAgentParameters(
                position=position,
                journey_id=journey_id,
                stage_id=stage_id,
                desiredSpeed=v0,
                radius=radius,
            )
        )
    ),
}


def build_agent_parameters(
    model_type: str,
    *,
    position: tuple[float, float],
    journey_id: int,
    stage_id: int,
    v0: float,
    radius: float,
) -> Any:
    """Construct an agent-parameters object for the given model."""
    try:
        builder = _AGENT_PARAMS_BUILDERS[model_type]
    except KeyError as exc:
        raise ValueError(
            f"No agent-parameters builder for model '{model_type}'"
        ) from exc
    return builder(
        position=position,
        journey_id=journey_id,
        stage_id=stage_id,
        v0=v0,
        radius=radius,
    )
