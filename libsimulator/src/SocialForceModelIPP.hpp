// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "SocialForceModelIPPData.hpp"
#include "CollisionGeometry.hpp"
#include "NeighborhoodSearch.hpp"
#include "OperationalModel.hpp"
#include "UniqueID.hpp"

struct GenericAgent;

class SocialForceModelIPP : public OperationalModel
{
public:
    using NeighborhoodSearchType = NeighborhoodSearch<GenericAgent>;

private:
    double _cutOffRadius{2.5};
    double bodyForce;
    double friction;

public:
    // Locomotion phase coupling constants
    static constexpr double LAMBDA_LOCOMOTION_2 = 0.5; // velocity coupling to ground support direction
    static constexpr double LAMBDA_LOCOMOTION_3 = 1.0; // velocity dissipation

    // Recovery phase coupling constants
    static constexpr double LAMBDA_RECOVERY_2 = 1.0; // ground support velocity coupling to upper body direction

    // Anthropometric scaling factors (multiplied by height [m])
    static constexpr double GS_SCALING_FACTOR =
        0.26 / (2 * 0.3 * 1.65); // foot length / (diameter * height)
    static constexpr double LEG_SCALING_FACTOR =
        0.5242; // shank(0.2522) + thigh(0.2269) + ankle(0.0451)

    static constexpr double g = 9.80665; // standard gravity [m/s^2]

    // Contact force parameters
    static constexpr double CONTACT_SCALE = 2.0;  // contact force amplitude
    static constexpr double CONTACT_RANGE = 0.5;  // contact force decay length [m]

    SocialForceModelIPP(double bodyForce_, double friction_);
    ~SocialForceModelIPP() override = default;
    OperationalModelType Type() const override;
    OperationalModelUpdate ComputeNewPosition(
        double dT,
        const GenericAgent& ped,
        const CollisionGeometry& geometry,
        const NeighborhoodSearchType& neighborhoodSearch) const override;
    void ApplyUpdate(const OperationalModelUpdate& update, GenericAgent& agent) const override;
    void CheckModelConstraint(
        const GenericAgent& agent,
        const NeighborhoodSearchType& neighborhoodSearch,
        const CollisionGeometry& geometry) const override;
    std::unique_ptr<OperationalModel> Clone() const override;

private:
    /**
     * Driving force acting on pedestrian <agent>
     * @param agent reference to Pedestrian
     *
     * @return vector with driving force of pedestrian
     */
    static Point DrivingForce(const GenericAgent& agent);
    /**
     *  Repulsive social force acting on pedestrian <ped1> from pedestrian <ped2>
     * @param ped1 reference to Pedestrian 1 on whom the force acts on
     * @param ped2 reference to Pedestrian 2, from whom the force originates
     * @return vector with the repulsive force
     */
    Point AgentSocialForce(const GenericAgent& ped1, const GenericAgent& ped2) const;
    /**
     *  Repulsive social force acting on pedestrian <agent> from line segment <segment>
     * @param agent reference to the Pedestrian on whom the force acts on
     * @param segment reference to line segment, from which the force originates
     * @return vector with the repulsive force
     */
    Point ObstacleSocialForce(const GenericAgent& agent, const LineSegment& segment) const;

    /**
     *  Repulsive contact acting on pedestrian <ped1> from pedestrian <ped2> at the upper body level
     * @param ped1 reference to Pedestrian 1 on whom the force acts on
     * @param ped2 reference to Pedestrian 2, from whom the force originates
     * @return vector with the repulsive force
     */
    Point AgentUpperBodyContactForce(const GenericAgent& ped1, const GenericAgent& ped2) const;

    /**
     *  Repulsive contact acting on pedestrian <ped1> from pedestrian <ped2> at the ground level bettwen support surfaces
     * @param ped1 reference to Pedestrian 1 on whom the force acts on
     * @param ped2 reference to Pedestrian 2, from whom the force originates
     * @return vector with the repulsive force
     */
    Point AgentGroundSupportContactForce(const GenericAgent& ped1, const GenericAgent& ped2) const;

    /**
     *  Repulsive contact force acting on pedestrian <agent> from line segment <segment> at the upper body level
     * @param agent reference to the Pedestrian on whom the force acts on
     * @param segment reference to line segment, from which the force originates
     * @return vector with the repulsive force
     */
    Point ObstacleUpperBodyContactForce(const GenericAgent& agent, const LineSegment& segment) const;

    /**
     *  Repulsive contact force acting on pedestrian <agent> from line segment <segment> at the ground level bettwen support surfaces
     * @param agent reference to the Pedestrian on whom the force acts on
     * @param segment reference to line segment, from which the force originates
     * @return vector with the repulsive force
     */
    Point ObstacleGroundSupportContactForce(const GenericAgent& agent, const LineSegment& segment) const;

    /**
     * calculates the social forces acting between <pt1> and <pt2>
     * @param pt1 Point on which the forces act
     * @param pt2 Point from which the forces originate
     * @param A Agent scale
     * @param B force distance
     * @param r radius
     * @param velocity velocity difference
     */
    Point SocialForceBetweenPoints(
        const Point pt1,
        const Point pt2,
        const double A,
        const double B,
        const double radius) const;


    /* calculates the contacts, normal repultions and friction forces acting between <pt1> and <pt2>
     * @param pt1 Point on which the forces act
     * @param pt2 Point from which the forces originate
     * @param r radius
     * @param velocity velocity difference
     */
    Point ContactForceBetweenPoints(
        const Point pt1,
        const Point pt2,
        const double radius,
        const Point velocity) const;
    /**
     *  exponential function that specifies the length of the pushing force between two points
     * @param A Agent scale
     * @param B force distance
     * @param r radius
     * @param distance distance between the two points
     * @return length of pushing force between the two points
     */
    static double PushingForceLength(double A, double B, double r, double distance);
};
