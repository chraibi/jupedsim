# SPDX-License-Identifier: LGPL-3.0-or-later

from dataclasses import dataclass

import jupedsim.native as py_jps


@dataclass(kw_only=True)
class SocialForceModelIPP:
    r"""Parameters for Social Force Model with Inverted Pendulum Paradigm.

    Two-level agent representation: upper body (center of mass) and
    ground support (feet/contact surface).

    Attributes:
        body_force: contact push strength k [kg s^-2]
        friction: contact friction strength kappa [kg m^-1 s^-1]
    """

    body_force: float = 120000
    friction: float = 240000


@dataclass(kw_only=True)
class SocialForceModelIPPAgentParameters:
    """Parameters required to create an Agent in the SocialForceModelIPP.

    Attributes:
        position: Position of the agent.
        orientation: Orientation of the agent.
        journey_id: Id of the journey the agent follows.
        stage_id: Id of the stage the agent targets.
        velocity: Current velocity of the agent.
        ground_support_position: Position of the ground support circle center.
        ground_support_velocity: Velocity of the ground support circle center.
        height: Height of the agent [m].
        mass: Mass of the agent [kg].
        desired_speed: Desired speed [m/s].
        reaction_time: Reaction time [s].
        agent_scale: Social force strength vs agents [N].
        obstacle_scale: Social force strength vs obstacles [N].
        force_distance: Social force range [m].
        radius: Upper body radius [m].
    """

    position: tuple[float, float] = (0.0, 0.0)
    orientation: tuple[float, float] = (0.0, 0.0)
    journey_id: int = -1
    stage_id: int = -1
    velocity: tuple[float, float] = (0.0, 0.0)
    ground_support_position: tuple[float, float] = (0.0, 0.0)
    ground_support_velocity: tuple[float, float] = (0.0, 0.0)
    height: float = 1.65
    mass: float = 80.0
    desired_speed: float = 0.8
    reaction_time: float = 0.5
    agent_scale: float = 2000.0
    obstacle_scale: float = 2000.0
    force_distance: float = 0.08
    radius: float = 0.3

    def as_native(
        self,
    ) -> py_jps.SocialForceModelIPPAgentParameters:
        return py_jps.SocialForceModelIPPAgentParameters(
            position=self.position,
            orientation=self.orientation,
            journey_id=self.journey_id,
            stage_id=self.stage_id,
            velocity=self.velocity,
            ground_support_position=self.ground_support_position,
            ground_support_velocity=self.ground_support_velocity,
            height=self.height,
            mass=self.mass,
            desired_speed=self.desired_speed,
            reaction_time=self.reaction_time,
            agent_scale=self.agent_scale,
            obstacle_scale=self.obstacle_scale,
            force_distance=self.force_distance,
            radius=self.radius,
        )


class SocialForceModelIPPState:
    def __init__(self, backing) -> None:
        self._obj = backing

    @property
    def velocity(self) -> tuple[float, float]:
        """Velocity of this agent."""
        return self._obj.velocity

    @velocity.setter
    def velocity(self, velocity):
        self._obj.velocity = velocity

    @property
    def ground_support_position(self) -> tuple[float, float]:
        """Ground support position of this agent."""
        return self._obj.ground_support_position

    @ground_support_position.setter
    def ground_support_position(self, ground_support_position):
        self._obj.ground_support_position = ground_support_position

    @property
    def ground_support_velocity(self) -> tuple[float, float]:
        """Ground support velocity of this agent."""
        return self._obj.ground_support_velocity

    @ground_support_velocity.setter
    def ground_support_velocity(self, ground_support_velocity):
        self._obj.ground_support_velocity = ground_support_velocity

    @property
    def height(self) -> float:
        """Height of this agent."""
        return self._obj.height

    @height.setter
    def height(self, height):
        self._obj.height = height

    @property
    def mass(self) -> float:
        """Mass of this agent."""
        return self._obj.mass

    @mass.setter
    def mass(self, mass):
        self._obj.mass = mass

    @property
    def desired_speed(self) -> float:
        """Desired speed of this agent."""
        return self._obj.desired_speed

    @desired_speed.setter
    def desired_speed(self, desired_speed):
        self._obj.desired_speed = desired_speed

    @property
    def reaction_time(self) -> float:
        """Reaction time of this agent."""
        return self._obj.reaction_time

    @reaction_time.setter
    def reaction_time(self, reaction_time):
        self._obj.reaction_time = reaction_time

    @property
    def agent_scale(self) -> float:
        """Social force strength vs agents."""
        return self._obj.agent_scale

    @agent_scale.setter
    def agent_scale(self, agent_scale):
        self._obj.agent_scale = agent_scale

    @property
    def obstacle_scale(self) -> float:
        """Social force strength vs obstacles."""
        return self._obj.obstacle_scale

    @obstacle_scale.setter
    def obstacle_scale(self, obstacle_scale):
        self._obj.obstacle_scale = obstacle_scale

    @property
    def force_distance(self) -> float:
        """Social force range."""
        return self._obj.force_distance

    @force_distance.setter
    def force_distance(self, force_distance):
        self._obj.force_distance = force_distance

    @property
    def radius(self) -> float:
        """Radius of this agent."""
        return self._obj.radius

    @radius.setter
    def radius(self, radius):
        self._obj.radius = radius
