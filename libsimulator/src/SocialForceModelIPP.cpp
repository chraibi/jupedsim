// SPDX-License-Identifier: LGPL-3.0-or-later
#include "SocialForceModelIPP.hpp"

#include "GenericAgent.hpp"
#include "Mathematics.hpp"
#include "NeighborhoodSearch.hpp"
#include "OperationalModelType.hpp"
#include "SimulationError.hpp"
#include "SocialForceModelIPPData.hpp"

#include <stdexcept>

SocialForceModelIPP::SocialForceModelIPP(double bodyForce_, double friction_)
    : bodyForce(bodyForce_), friction(friction_) {};

OperationalModelType SocialForceModelIPP::Type() const
{
    return OperationalModelType::SOCIAL_FORCE_IPP;
}

std::unique_ptr<OperationalModel> SocialForceModelIPP::Clone() const
{
    return std::make_unique<SocialForceModelIPP>(*this);
}

OperationalModelUpdate SocialForceModelIPP::ComputeNewPosition(
    double dT,
    const GenericAgent& ped,
    const CollisionGeometry& geometry,
    const NeighborhoodSearchType& neighborhoodSearch) const
{
    const auto& model = std::get<SocialForceModelIPPData>(ped.model);
    SocialForceModelIPPUpdate update{};

    const auto neighborhood = neighborhoodSearch.GetNeighboringAgents(ped.pos, this->_cutOffRadius);
    const auto& walls = geometry.LineSegmentsInApproxDistanceTo(ped.pos);

    // Collision avoidance: Social Forces
    auto social_forces = DrivingForce(ped);
    Point F_rep_agents;
    for(const auto& neighbor : neighborhood) {
        if(neighbor.id == ped.id) {
            continue;
        }
        F_rep_agents += AgentSocialForce(ped, neighbor);
    }
    social_forces += F_rep_agents / model.mass;

    const auto F_social_obstacles = std::accumulate(
        walls.cbegin(),
        walls.cend(),
        Point(0, 0),
        [this, &ped](const auto& acc, const auto& element) {
            return acc + ObstacleSocialForce(ped, element);
        });
    social_forces += F_social_obstacles / model.mass;

    // Physical interactions: upper body contacts
    Point upper_body_contact_forces;
    for(const auto& neighbor : neighborhood) {
        if(neighbor.id == ped.id) {
            continue;
        }
        upper_body_contact_forces += AgentUpperBodyContactForce(ped, neighbor);
    }
    upper_body_contact_forces += std::accumulate(
        walls.cbegin(),
        walls.cend(),
        Point(0, 0),
        [this, &ped](const auto& acc, const auto& element) {
            return acc + ObstacleUpperBodyContactForce(ped, element);
        });

    // Physical interactions: ground support contacts
    Point ground_support_contact_forces;
    for(const auto& neighbor : neighborhood) {
        if(neighbor.id == ped.id) {
            continue;
        }
        ground_support_contact_forces += AgentGroundSupportContactForce(ped, neighbor);
    }
    ground_support_contact_forces += std::accumulate(
        walls.cbegin(),
        walls.cend(),
        Point(0, 0),
        [this, &ped](const auto& acc, const auto& element) {
            return acc + ObstacleGroundSupportContactForce(ped, element);
        });

    // Unit vector from ground support pointing to upper body
    Point e_gs_ub = (ped.pos - model.ground_support_position).Normalized();

    // Upper body dynamics
    update.velocity = model.velocity +
                      (social_forces + upper_body_contact_forces +
                       (e_gs_ub - model.velocity) * LAMBDA_LOCOMOTION_2 -
                       model.velocity * LAMBDA_LOCOMOTION_3) *
                          dT;
    update.position = ped.pos + update.velocity * dT;

    // Ground support dynamics
    update.ground_support_velocity =
        model.ground_support_velocity +
        (ground_support_contact_forces +
         (e_gs_ub - model.ground_support_velocity) * LAMBDA_RECOVERY_2) *
            dT;
    update.ground_support_position =
        model.ground_support_position + update.ground_support_velocity * dT;

    return update;
}

void SocialForceModelIPP::ApplyUpdate(const OperationalModelUpdate& update, GenericAgent& agent)
    const
{
    auto& model = std::get<SocialForceModelIPPData>(agent.model);
    const auto& upd = std::get<SocialForceModelIPPUpdate>(update);
    agent.pos = upd.position;
    agent.orientation = upd.velocity.Normalized();
    model.velocity = upd.velocity;
    model.ground_support_position = upd.ground_support_position;
    model.ground_support_velocity = upd.ground_support_velocity;
}

void SocialForceModelIPP::CheckModelConstraint(
    const GenericAgent& agent,
    const NeighborhoodSearchType& neighborhoodSearch,
    const CollisionGeometry& geometry) const
{
    auto throwIfNegative = [](double value, std::string name) {
        if(value < 0) {
            throw SimulationError(
                "Model constraint violation: {} {} not in allowed range, "
                "{} needs to be positive",
                name,
                value,
                name);
        }
    };

    const auto& model = std::get<SocialForceModelIPPData>(agent.model);

    throwIfNegative(model.mass, "mass");
    throwIfNegative(model.desiredSpeed, "desired speed");
    throwIfNegative(model.reactionTime, "reaction time");
    throwIfNegative(model.radius, "radius");

    const auto neighbors = neighborhoodSearch.GetNeighboringAgents(agent.pos, 2);
    for(const auto& neighbor : neighbors) {
        const auto distance = (agent.pos - neighbor.pos).Norm();
        if(model.radius >= distance) {
            throw SimulationError(
                "Model constraint violation: Agent {} too close to agent {}: distance {}, "
                "radius {}",
                agent.pos,
                neighbor.pos,
                distance,
                model.radius);
        }
    }
    const auto maxRadius = model.radius / 2;
    const auto lineSegments = geometry.LineSegmentsInDistanceTo(maxRadius, agent.pos);
    if(std::begin(lineSegments) != std::end(lineSegments)) {
        throw SimulationError(
            "Model constraint violation: Agent {} too close to geometry boundaries, distance <= "
            "{}/2",
            agent.pos,
            model.radius);
    }
}

Point SocialForceModelIPP::DrivingForce(const GenericAgent& agent)
{
    const auto& model = std::get<SocialForceModelIPPData>(agent.model);
    const Point e0 = (agent.destination - agent.pos).Normalized();
    return (e0 * model.desiredSpeed - model.velocity) / model.reactionTime;
}

double SocialForceModelIPP::PushingForceLength(double A, double B, double r, double distance)
{
    return A * exp((r - distance) / B);
}

Point SocialForceModelIPP::AgentSocialForce(
    const GenericAgent& ped1,
    const GenericAgent& ped2) const
{
    const auto& model1 = std::get<SocialForceModelIPPData>(ped1.model);
    const auto& model2 = std::get<SocialForceModelIPPData>(ped2.model);
    const double radiuses_sum = model1.radius + model2.radius;
    return SocialForceBetweenPoints(
        ped1.pos, ped2.pos, model1.agentScale, model1.forceDistance, radiuses_sum);
}

Point SocialForceModelIPP::ObstacleSocialForce(
    const GenericAgent& agent,
    const LineSegment& segment) const
{
    const auto& model = std::get<SocialForceModelIPPData>(agent.model);
    const Point pt = segment.ShortestPoint(agent.pos);
    return SocialForceBetweenPoints(
        agent.pos, pt, model.obstacleScale, model.forceDistance, model.radius);
}

Point SocialForceModelIPP::AgentUpperBodyContactForce(
    const GenericAgent& ped1,
    const GenericAgent& ped2) const
{
    const auto& model1 = std::get<SocialForceModelIPPData>(ped1.model);
    const auto& model2 = std::get<SocialForceModelIPPData>(ped2.model);
    const double radiuses_sum = model1.radius + model2.radius;
    return ContactForceBetweenPoints(
        ped1.pos, ped2.pos, radiuses_sum, model2.velocity - model1.velocity);
}

Point SocialForceModelIPP::AgentGroundSupportContactForce(
    const GenericAgent& ped1,
    const GenericAgent& ped2) const
{
    const auto& model1 = std::get<SocialForceModelIPPData>(ped1.model);
    const auto& model2 = std::get<SocialForceModelIPPData>(ped2.model);
    const double radiuses_sum = model1.radius * GS_SCALING_FACTOR * model1.height +
                                model2.radius * GS_SCALING_FACTOR * model2.height;
    return ContactForceBetweenPoints(
        model1.ground_support_position,
        model2.ground_support_position,
        radiuses_sum,
        model2.ground_support_velocity - model1.ground_support_velocity);
}

Point SocialForceModelIPP::ObstacleUpperBodyContactForce(
    const GenericAgent& agent,
    const LineSegment& segment) const
{
    const auto& model = std::get<SocialForceModelIPPData>(agent.model);
    const Point pt = segment.ShortestPoint(agent.pos);
    return ContactForceBetweenPoints(agent.pos, pt, model.radius, model.velocity);
}

Point SocialForceModelIPP::ObstacleGroundSupportContactForce(
    const GenericAgent& agent,
    const LineSegment& segment) const
{
    const auto& model = std::get<SocialForceModelIPPData>(agent.model);
    const Point pt = segment.ShortestPoint(model.ground_support_position);
    return ContactForceBetweenPoints(
        model.ground_support_position,
        pt,
        model.radius * GS_SCALING_FACTOR * model.height,
        model.ground_support_velocity);
}

Point SocialForceModelIPP::SocialForceBetweenPoints(
    const Point pt1,
    const Point pt2,
    const double A,
    const double B,
    const double radiuses_sum) const
{
    const double dist = (pt1 - pt2).Norm();
    double pushing_force_norm = PushingForceLength(A, B, radiuses_sum, dist);
    const Point n_ij = (pt1 - pt2).Normalized();
    return n_ij * pushing_force_norm;
}

Point SocialForceModelIPP::ContactForceBetweenPoints(
    const Point pt1,
    const Point pt2,
    const double radiuses_sum,
    const Point velocity) const
{
    const double dist = (pt1 - pt2).Norm();
    double pushing_force_norm = 0;
    double friction_force_norm = 0;
    const Point n_ij = (pt1 - pt2).Normalized();
    const Point tangent = n_ij.Rotate90Deg();
    if(dist < radiuses_sum) {
        pushing_force_norm = this->bodyForce * (radiuses_sum - dist);
        friction_force_norm =
            this->friction * (radiuses_sum - dist) * (velocity.ScalarProduct(tangent));
    }
    return n_ij * pushing_force_norm + tangent * friction_force_norm;
}
