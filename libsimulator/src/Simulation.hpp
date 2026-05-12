// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "AgentRemovalSystem.hpp"
#include "BuildingTopology.hpp"
#include "CollisionGeometry.hpp"
#include "GenericAgent.hpp"
#include "Journey.hpp"
#include "NeighborhoodSearch.hpp"
#include "OperationalDecisionSystem.hpp"
#include "OperationalModel.hpp"
#include "OperationalModelType.hpp"
#include "Point.hpp"
#include "Polygon.hpp"
#include "RoutingEngine.hpp"
#include "SimulationClock.hpp"
#include "Stage.hpp"
#include "StageDescription.hpp"
#include "StageManager.hpp"
#include "StageSystem.hpp"
#include "StrategicalDesicionSystem.hpp"
#include "TacticalDecisionSystem.hpp"
#include "Timing.hpp"
#include "Tracing.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <tuple>
#include <unordered_map>
#include <vector>

class Simulation
{
    SimulationClock _clock;
    StrategicalDecisionSystem _stategicalDecisionSystem{};
    TacticalDecisionSystem _tacticalDecisionSystem{};
    OperationalDecisionSystem _operationalDecisionSystem;
    AgentRemovalSystem<GenericAgent> _agentRemovalSystem{};
    StageManager _stageManager{};
    StageSystem _stageSystem{};
    // One neighborhood grid per level. Agents are partitioned by
    // currentLevel each iteration; cross-level neighbor interactions are
    // intentionally suppressed (floors share an (x,y) footprint).
    std::unordered_map<CollisionGeometry::ID, NeighborhoodSearch<GenericAgent>>
        _neighborhoodSearches{};
    std::unordered_map<
        CollisionGeometry::ID,
        std::tuple<std::unique_ptr<CollisionGeometry>, std::unique_ptr<RoutingEngine>>>
        geometries{};
    RoutingEngine* _routingEngine;
    CollisionGeometry* _geometry;
    // The level new agents are assigned to when they enter the simulation
    // without an explicit level. Also the level the legacy SwitchGeometry
    // path operates on.
    CollisionGeometry::ID _primaryLevel{CollisionGeometry::ID::Invalid};
    BuildingTopology _topology{};
    // Per-level elevation in world units. Used by the trajectory writer to
    // place each floor at the right z for 3D visualization. Has no effect
    // on the 2D simulation itself.
    std::unordered_map<CollisionGeometry::ID, double> _levelElevations{};
    std::vector<GenericAgent> _agents;
    std::vector<GenericAgent::ID> _removedAgentsInLastIteration;
    std::unordered_map<Journey::ID, std::unique_ptr<Journey>> _journeys;
    Timer _timer{};
    enum LogLevel { General = 1, Detailed = 2, Debug = 3 };

public:
    Simulation(
        std::unique_ptr<OperationalModel>&& operationalModel,
        std::unique_ptr<CollisionGeometry>&& geometry,
        double dT);
    Simulation(const Simulation& other) = delete;
    Simulation& operator=(const Simulation& other) = delete;
    Simulation(Simulation&& other) = delete;
    Simulation& operator=(Simulation&& other) = delete;
    ~Simulation() = default;
    const SimulationClock& Clock() const;
    void SetTracing(bool on);
    void Iterate();
    Journey::ID AddJourney(const std::map<BaseStage::ID, TransitionDescription>& stages);
    BaseStage::ID AddStage(const StageDescription stageDescription);
    void MarkAgentForRemoval(GenericAgent::ID id);
    const std::vector<GenericAgent::ID>& RemovedAgents() const;
    size_t AgentCount() const;
    double ElapsedTime() const;
    double DT() const;
    void
    SwitchAgentJourney(GenericAgent::ID agent_id, Journey::ID journey_id, BaseStage::ID stage_id);
    uint64_t Iteration() const;
    std::vector<GenericAgent::ID> AgentsInRange(Point p, double distance);
    /// Returns IDs of all agents inside the defined polygon
    /// @param polygon Required to be a simple convex polygon with CCW ordering.
    std::vector<GenericAgent::ID> AgentsInPolygon(const std::vector<Point>& polygon);
    GenericAgent::ID AddAgent(GenericAgent agent);
    const GenericAgent& Agent(GenericAgent::ID id) const;
    GenericAgent& Agent(GenericAgent::ID id);
    std::vector<GenericAgent>& Agents();
    OperationalModelType ModelType() const;
    StageProxy Stage(BaseStage::ID stageId);
    CollisionGeometry Geo() const;
    CollisionGeometry::ID PrimaryLevel() const { return _primaryLevel; }
    void SwitchGeometry(std::unique_ptr<CollisionGeometry>&& geometry);
    // Register an additional level (a floor or a stair). Returns its id.
    // The first geometry passed to the Simulation ctor is the primary level;
    // subsequent ones are added via this method. `elevation` is the world-z
    // of the level; used only by the trajectory writer for 3D viz.
    CollisionGeometry::ID
    AddLevel(std::unique_ptr<CollisionGeometry>&& geometry, double elevation = 0.0);
    // Sorted list of registered level ids (primary first).
    std::vector<CollisionGeometry::ID> LevelIds() const;
    double LevelElevation(CollisionGeometry::ID id) const;
    const CollisionGeometry& LevelGeometry(CollisionGeometry::ID id) const;
    const BuildingTopology& Topology() const { return _topology; }
    // Connect two levels via a landing portal. An agent on `from` whose
    // position lies inside `polyFrom` will be transferred to `to` (its
    // (x,y) is preserved; the landings are expected to overlap in plan).
    void AddLanding(
        CollisionGeometry::ID from,
        const std::vector<Point>& polyFrom,
        CollisionGeometry::ID to,
        const std::vector<Point>& polyTo);
    void PushTimer(const std::string_view name, size_t probe_log_level = 0);
    void PopTimer(const std::string_view name);
    void SetTimerLogLevel(int level) { _timer.setLogLevel(level); };
    TimerEntry::duration_type GetTimerDuration(const std::string_view name) const;
    std::map<std::string, TimerEntry::duration_type> GetTimerDurations() const;

private:
    void ValidateGeometry(const std::unique_ptr<CollisionGeometry>& geometry) const;
};
