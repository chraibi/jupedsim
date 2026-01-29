// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "CollisionFreeSpeedModelData.hpp"
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
    // Models parameters and constants;
    // static constexpr double LAMBDA_LOCOMOTION_1 = 5; // model parameter that ajust the control on position during Locomotion phase
    static constexpr double LAMBDA_LOCOMOTION_2 = 0.5; // model parameter that ajust the control on velocity during Locomotion phase
    static constexpr double LAMBDA_LOCOMOTION_3 = 1; // model parameter that ajust the dissipation term during Locomotion phase

    // static constexpr double LAMBDA_RECOVERY_1 = 5; // model parameter that ajust the control on position during Recovery phase
    static constexpr double LAMBDA_RECOVERY_2 = 1; // model parameter that ajust the control on velocity during Recovery phase
    // static constexpr double LAMBDA_RECOVERY_3 = 1; // model parameter that ajust the dissipation term during Recovery phase

    // Scalin factors multiplied by the height [m] gives
    static constexpr double GS_SCALING_FACTOR = 0.26 /(2*0.3*1.65) ; // Ground support circle radius in meters. 
                                                                    //  Based on average adult foot length (26cm)
    static constexpr double LEG_SCALING_FACTOR = 0.5242; // Leg length in metters. 0.2522 (shank) + 0.2269 (thigh) + 0.0451 (ankle)

    static constexpr double g = 9.80665; // standard gravity acceleration in m/s^2

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
