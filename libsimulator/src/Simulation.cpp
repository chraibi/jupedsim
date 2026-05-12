// SPDX-License-Identifier: LGPL-3.0-or-later
#include "Simulation.hpp"

#include "CollisionGeometry.hpp"
#include "GeneralizedCentrifugalForceModelData.hpp"
#include "GenericAgent.hpp"
#include "GeometrySwitchError.hpp"
#include "IteratorPair.hpp"
#include "Journey.hpp"
#include "OperationalModel.hpp"
#include "OperationalModelType.hpp"
#include "Point.hpp"
#include "Polygon.hpp"
#include "RoutingEngine.hpp"
#include "SimulationClock.hpp"
#include "SimulationError.hpp"
#include "Stage.hpp"
#include "StageDescription.hpp"
#include "Tracing.hpp"
#include "Visitor.hpp"

#include <fmt/core.h>
#include <fmt/format.h>
#include <fmt/ranges.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <variant>
#include <vector>

Simulation::Simulation(
    std::unique_ptr<OperationalModel>&& operationalModel,
    std::unique_ptr<CollisionGeometry>&& geometry,
    double dT)
    : _clock(dT), _operationalDecisionSystem(std::move(operationalModel))
{
    const auto p = geometry->Polygon();
    const auto& [tup, res] = geometries.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(geometry->Id()),
        std::forward_as_tuple(std::move(geometry), std::make_unique<RoutingEngine>(p)));
    if(!res) {
        throw SimulationError("Internal error");
    }
    _geometry = std::get<0>(tup->second).get();
    _routingEngine = std::get<1>(tup->second).get();
    _primaryLevel = _geometry->Id();
    _neighborhoodSearches.try_emplace(_primaryLevel, 2.2);
    _levelElevations[_primaryLevel] = 0.0;
}

CollisionGeometry::ID
Simulation::AddLevel(std::unique_ptr<CollisionGeometry>&& geometry, double elevation)
{
    JPS_TRACE_FUNC;
    const auto id = geometry->Id();
    const auto p = geometry->Polygon();
    const auto& [_, inserted] = geometries.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(id),
        std::forward_as_tuple(std::move(geometry), std::make_unique<RoutingEngine>(p)));
    if(!inserted) {
        throw SimulationError("Level already registered");
    }
    _neighborhoodSearches.try_emplace(id, 2.2);
    _levelElevations[id] = elevation;
    return id;
}

std::vector<CollisionGeometry::ID> Simulation::LevelIds() const
{
    std::vector<CollisionGeometry::ID> ids;
    ids.reserve(geometries.size());
    ids.push_back(_primaryLevel);
    for(const auto& [id, _] : geometries) {
        if(id != _primaryLevel) {
            ids.push_back(id);
        }
    }
    return ids;
}

double Simulation::LevelElevation(CollisionGeometry::ID id) const
{
    const auto it = _levelElevations.find(id);
    if(it == std::end(_levelElevations)) {
        throw SimulationError("Unknown level id: {}", id);
    }
    return it->second;
}

const CollisionGeometry& Simulation::LevelGeometry(CollisionGeometry::ID id) const
{
    const auto it = geometries.find(id);
    if(it == std::end(geometries)) {
        throw SimulationError("Unknown level id: {}", id);
    }
    return *std::get<0>(it->second);
}

void Simulation::AddLanding(
    CollisionGeometry::ID from,
    const std::vector<Point>& polyFrom,
    CollisionGeometry::ID to,
    const std::vector<Point>& polyTo)
{
    JPS_TRACE_FUNC;
    if(geometries.find(from) == std::end(geometries)) {
        throw SimulationError("Unknown level id (from): {}", from);
    }
    if(geometries.find(to) == std::end(geometries)) {
        throw SimulationError("Unknown level id (to): {}", to);
    }
    _topology.AddLanding(from, polyFrom, to, polyTo);
}
const SimulationClock& Simulation::Clock() const
{
    return _clock;
}

void Simulation::SetTracing(bool status)
{
    if(status) {
        Profiler::instance().enable();
    } else {
        Profiler::instance().disable();
    }
};

void Simulation::Iterate()
{
    JPS_SCOPED_TIMER_AND_TRACE(_timer, "Total Iteration", General);

    {
        JPS_SCOPED_TIMER_AND_TRACE(_timer, "Agent Removal System", Detailed);
        _agentRemovalSystem.Run(_agents, _removedAgentsInLastIteration, _stageManager);
    }

    {
        JPS_SCOPED_TIMER_AND_TRACE(_timer, "Neighborhood Search", Detailed);
        // Partition agents by currentLevel into the per-level grids.
        std::unordered_map<CollisionGeometry::ID, std::vector<GenericAgent>> bucketed;
        for(const auto& a : _agents) {
            bucketed[a.currentLevel].push_back(a);
        }
        for(auto& [id, ns] : _neighborhoodSearches) {
            ns.Update(bucketed[id]);
        }
    }

    {
        JPS_SCOPED_TIMER_AND_TRACE(_timer, "Stage System", Detailed);
        // Stages are not yet level-aware; queue/waiting-set updates use the
        // primary level's neighborhood + geometry. Multi-level stage
        // ownership lands with the per-stage `level` field.
        _stageSystem.Run(
            _stageManager, _neighborhoodSearches.at(_primaryLevel), *_geometry);
    }

    {
        JPS_SCOPED_TIMER_AND_TRACE(_timer, "Strategical Decision System", General);
        _stategicalDecisionSystem.Run(_journeys, _agents, _stageManager);
    }

    {
        JPS_SCOPED_TIMER_AND_TRACE(_timer, "Tactical Decision System", General);
        _tacticalDecisionSystem.Run(
            [this](CollisionGeometry::ID id) -> RoutingEngine& {
                return *std::get<1>(geometries.at(id));
            },
            _agents);
    }

    {
        JPS_SCOPED_TIMER_AND_TRACE(_timer, "Operational Decision System", General);
        _operationalDecisionSystem.Run(
            _clock.dT(),
            _clock.ElapsedTime(),
            [this](CollisionGeometry::ID id) -> const CollisionGeometry& {
                return *std::get<0>(geometries.at(id));
            },
            [this](CollisionGeometry::ID id) -> const NeighborhoodSearch<GenericAgent>& {
                return _neighborhoodSearches.at(id);
            },
            _agents);
    }

    {
        JPS_SCOPED_TIMER_AND_TRACE(_timer, "Level Switch", Detailed);
        // Level crossing: an agent in a landing polygon on its current
        // level is transferred to the connected level. (x,y) is preserved;
        // landings overlap in plan. Re-bucketing into the destination grid
        // happens at the start of the next iteration.
        for(auto& agent : _agents) {
            const auto* landing = _topology.FindLanding(agent.currentLevel, agent.pos);
            if(landing != nullptr) {
                agent.currentLevel = landing->to;
            }
        }
    }
    _clock.Advance();
}

Journey::ID Simulation::AddJourney(const std::map<BaseStage::ID, TransitionDescription>& stages)
{
    JPS_SCOPED_TIMER_AND_TRACE(_timer, "Add Journey", Detailed);
    std::map<BaseStage::ID, JourneyNode> nodes;
    bool containsDirectSteering =
        std::find_if(std::begin(stages), std::end(stages), [this](auto const& pair) {
            return std::holds_alternative<DirectSteeringProxy>(Stage(pair.first));
        }) != std::end(stages);

    if(containsDirectSteering && stages.size() > 1) {
        throw SimulationError(
            "Journeys containing a DirectSteeringStage, may only contain this stage.");
    }

    std::transform(
        std::begin(stages),
        std::end(stages),
        std::inserter(nodes, std::end(nodes)),
        [this](auto const& pair) -> std::pair<BaseStage::ID, JourneyNode> {
            const auto& [id, desc] = pair;
            auto stage = _stageManager.Stage(id);
            return {
                id,
                JourneyNode{
                    stage,
                    std::visit(
                        overloaded{
                            [stage](
                                const NonTransitionDescription&) -> std::unique_ptr<Transition> {
                                return std::make_unique<FixedTransition>(stage);
                            },
                            [this](const FixedTransitionDescription& d)
                                -> std::unique_ptr<Transition> {
                                return std::make_unique<FixedTransition>(
                                    _stageManager.Stage(d.NextId()));
                            },
                            [this](const RoundRobinTransitionDescription& d)
                                -> std::unique_ptr<Transition> {
                                std::vector<std::tuple<BaseStage*, uint64_t>> weightedStages{};
                                weightedStages.reserve(d.WeightedStages().size());

                                std::transform(
                                    std::begin(d.WeightedStages()),
                                    std::end(d.WeightedStages()),
                                    std::back_inserter(weightedStages),
                                    [this](auto const& pair) -> std::tuple<BaseStage*, uint64_t> {
                                        const auto& [id, weight] = pair;
                                        return {_stageManager.Stage(id), weight};
                                    });

                                return std::make_unique<RoundRobinTransition>(weightedStages);
                            },
                            [this](const LeastTargetedTransitionDescription& d)
                                -> std::unique_ptr<Transition> {
                                std::vector<BaseStage*> candidates{};
                                candidates.reserve(d.TargetCandidates().size());

                                std::transform(
                                    std::begin(d.TargetCandidates()),
                                    std::end(d.TargetCandidates()),
                                    std::back_inserter(candidates),
                                    [this](auto const& id) -> BaseStage* {
                                        return _stageManager.Stage(id);
                                    });

                                return std::make_unique<LeastTargetedTransition>(candidates);
                            }},
                        desc)}};
        });

    auto journey = std::make_unique<Journey>(std::move(nodes));
    const auto id = journey->Id();
    _journeys.emplace(id, std::move(journey));
    return id;
}

BaseStage::ID Simulation::AddStage(const StageDescription stageDescription)
{
    JPS_SCOPED_TIMER_AND_TRACE(_timer, "Add Stage", Detailed);
    std::visit(
        overloaded{
            [this](const WaypointDescription& d) -> void {
                if(!this->_geometry->InsideGeometry(d.position)) {
                    throw SimulationError("WayPoint {} not inside walkable area", d.position);
                }
            },
            [this](const ExitDescription& d) -> void {
                if(!this->_geometry->InsideGeometry(d.polygon.Centroid())) {
                    throw SimulationError("Exit {} not inside walkable area", d.polygon.Centroid());
                }
            },
            [this](const NotifiableWaitingSetDescription& d) -> void {
                for(const auto& point : d.slots) {
                    if(!this->_geometry->InsideGeometry(point)) {
                        throw SimulationError(
                            "NotifiableWaitingSet point {} not inside walkable area", point);
                    }
                }
            },
            [this](const NotifiableQueueDescription& d) -> void {
                for(const auto& point : d.slots) {
                    if(!this->_geometry->InsideGeometry(point)) {
                        throw SimulationError(
                            "NotifiableQueue point {} not inside walkable area", point);
                    }
                }
            },
            [](const DirectSteeringDescription&) -> void {

            }},
        stageDescription);

    return _stageManager.AddStage(stageDescription, _removedAgentsInLastIteration);
}

GenericAgent::ID Simulation::AddAgent(GenericAgent agent)
{
    JPS_SCOPED_TIMER_AND_TRACE(_timer, "Add Agent", Detailed);
    if(agent.currentLevel == CollisionGeometry::ID::Invalid) {
        agent.currentLevel = _primaryLevel;
    }
    const auto levelIter = geometries.find(agent.currentLevel);
    if(levelIter == std::end(geometries)) {
        throw SimulationError("Agent assigned to unknown level: {}", agent.currentLevel);
    }
    const auto& levelGeometry = *std::get<0>(levelIter->second);
    if(!levelGeometry.InsideGeometry(agent.pos)) {
        throw SimulationError("Agent {} not inside walkable area", agent.pos);
    }
    if(_journeys.count(agent.journeyId) == 0) {
        throw SimulationError("Unknown journey id: {}", agent.journeyId);
    }

    if(!_journeys.at(agent.journeyId)->ContainsStage(agent.stageId)) {
        throw SimulationError("Unknown stage id: {}", agent.stageId);
    }

    if(std::holds_alternative<GeneralizedCentrifugalForceModelData>(agent.model))
        if(agent.orientation.isZeroLength()) {
            throw SimulationError(
                "Orientation is invalid: {}. Length should be 1.", agent.orientation);
        }

    agent.orientation = agent.orientation.Normalized();
    auto& ns = _neighborhoodSearches.at(agent.currentLevel);
    _operationalDecisionSystem.ValidateAgent(agent, ns, levelGeometry);

    _stageManager.HandleNewAgent(agent.stageId);
    _agents.emplace_back(std::move(agent));
    ns.AddAgent(_agents.back());

    auto v = IteratorPair(std::prev(std::end(_agents)), std::end(_agents));
    _stategicalDecisionSystem.Run(_journeys, v, _stageManager);
    _tacticalDecisionSystem.Run(
        [this](CollisionGeometry::ID id) -> RoutingEngine& {
            return *std::get<1>(geometries.at(id));
        },
        v);
    return _agents.back().id.getID();
}

void Simulation::MarkAgentForRemoval(GenericAgent::ID id)
{
    JPS_TRACE_FUNC;
    const auto iter = std::find_if(
        std::begin(_agents), std::end(_agents), [id](auto& agent) { return agent.id == id; });
    if(iter == std::end(_agents)) {
        throw SimulationError("Unknown agent id {}", id);
    }

    _removedAgentsInLastIteration.push_back(id);
}

const GenericAgent& Simulation::Agent(GenericAgent::ID id) const
{
    JPS_TRACE_FUNC;
    const auto iter =
        std::find_if(_agents.begin(), _agents.end(), [id](auto& ped) { return id == ped.id; });
    if(iter == _agents.end()) {
        throw SimulationError("Trying to access unknown Agent {}", id);
    }
    return *iter;
}

GenericAgent& Simulation::Agent(GenericAgent::ID id)
{
    JPS_TRACE_FUNC;
    const auto iter =
        std::find_if(_agents.begin(), _agents.end(), [id](auto& ped) { return id == ped.id; });
    if(iter == _agents.end()) {
        throw SimulationError("Trying to access unknown Agent {}", id);
    }
    return *iter;
}

const std::vector<GenericAgent::ID>& Simulation::RemovedAgents() const
{
    return _removedAgentsInLastIteration;
}

double Simulation::ElapsedTime() const
{
    return _clock.ElapsedTime();
}

double Simulation::DT() const
{
    return _clock.dT();
}

uint64_t Simulation::Iteration() const
{
    return _clock.Iteration();
}

size_t Simulation::AgentCount() const
{
    return _agents.size();
}

std::vector<GenericAgent>& Simulation::Agents()
{
    return _agents;
};

void Simulation::SwitchAgentJourney(
    GenericAgent::ID agent_id,
    Journey::ID journey_id,
    BaseStage::ID stage_id)
{
    JPS_TRACE_FUNC;
    const auto find_iter = _journeys.find(journey_id);
    if(find_iter == std::end(_journeys)) {
        throw SimulationError("Unknown Journey id {}", journey_id);
    }
    auto& journey = find_iter->second;
    if(!journey->ContainsStage(stage_id)) {
        throw SimulationError("Stage {} not part of Journey {}", stage_id, journey_id);
    }
    auto& agent = Agent(agent_id);
    agent.journeyId = journey_id;
    _stageManager.MigrateAgent(agent.stageId, stage_id);
    agent.stageId = stage_id;
}

std::vector<GenericAgent::ID> Simulation::AgentsInRange(Point p, double distance)
{
    JPS_SCOPED_TIMER_AND_TRACE(_timer, "Agents in Range", Debug);
    // Spatial query unions across levels; the caller filters by level if it
    // cares (AgentsInRange has no level argument today).
    std::vector<GenericAgent::ID> neighborIds{};
    for(const auto& [_, ns] : _neighborhoodSearches) {
        const auto neighbors = ns.GetNeighboringAgents(p, distance);
        neighborIds.reserve(neighborIds.size() + neighbors.size());
        for(const auto& a : neighbors) {
            neighborIds.push_back(a.id);
        }
    }
    return neighborIds;
}

std::vector<GenericAgent::ID> Simulation::AgentsInPolygon(const std::vector<Point>& polygon)
{
    JPS_SCOPED_TIMER_AND_TRACE(_timer, "Agents in Polygon", Debug);
    const Polygon poly{polygon};
    if(!poly.IsConvex()) {
        throw SimulationError("Polygon needs to be simple and convex");
    }
    const auto [p, dist] = poly.ContainingCircle();

    std::vector<GenericAgent::ID> result{};
    for(const auto& [_, ns] : _neighborhoodSearches) {
        const auto candidates = ns.GetNeighboringAgents(p, dist);
        for(const auto& agent : candidates) {
            if(poly.IsInside(agent.pos)) {
                result.push_back(agent.id);
            }
        }
    }
    return result;
}

OperationalModelType Simulation::ModelType() const
{
    return _operationalDecisionSystem.ModelType();
}

StageProxy Simulation::Stage(BaseStage::ID stageId)
{
    return _stageManager.Stage(stageId)->Proxy(this);
}
CollisionGeometry Simulation::Geo() const
{
    return *_geometry;
}

void Simulation::SwitchGeometry(std::unique_ptr<CollisionGeometry>&& geometry)
{
    JPS_TRACE_FUNC;
    ValidateGeometry(geometry);
    if(const auto& iter = geometries.find(geometry->Id()); iter != std::end(geometries)) {
        _geometry = std::get<0>(iter->second).get();
        _routingEngine = std::get<1>(iter->second).get();
    } else {
        const auto p = geometry->Polygon();
        const auto& [tup, res] = geometries.emplace(
            std::piecewise_construct,
            std::forward_as_tuple(geometry->Id()),
            std::forward_as_tuple(std::move(geometry), std::make_unique<RoutingEngine>(p)));
        if(!res) {
            throw SimulationError("Internal error");
        }
        _geometry = std::get<0>(tup->second).get();
        _routingEngine = std::get<1>(tup->second).get();
    }
    _primaryLevel = _geometry->Id();
    for(auto& agent : _agents) {
        agent.currentLevel = _primaryLevel;
    }
}

void Simulation::ValidateGeometry(const std::unique_ptr<CollisionGeometry>& geometry) const
{
    JPS_TRACE_FUNC;
    std::vector<GenericAgent::ID> faultyAgents;
    for(const auto& agent : _agents) {
        if(const auto find_iter = std::find(
               std::begin(_removedAgentsInLastIteration),
               std::end(_removedAgentsInLastIteration),
               agent.id);
           find_iter != std::end(_removedAgentsInLastIteration)) {
            continue;
        }

        if(!geometry->InsideGeometry(agent.pos)) {
            faultyAgents.push_back(agent.id);
        }
    }

    std::vector<BaseStage::ID> faultyStages;
    for(const auto& [_, journey] : _journeys) {
        for(const auto& [stageId, node] : journey->Stages()) {

            if(auto exit = dynamic_cast<Exit*>(node.stage); exit != nullptr) {
                if(!geometry->InsideGeometry(exit->Position().Centroid())) {
                    faultyStages.push_back(stageId);
                }
            } else if(auto waypoint = dynamic_cast<Waypoint*>(node.stage); waypoint != nullptr) {
                if(!geometry->InsideGeometry(waypoint->Position())) {
                    faultyStages.push_back(stageId);
                }
            } else if(auto queue = dynamic_cast<NotifiableQueue*>(node.stage); queue != nullptr) {
                for(const auto& point : queue->Slots()) {
                    if(!geometry->InsideGeometry(point)) {
                        faultyStages.push_back(stageId);
                    }
                }
            } else if(auto waitingset = dynamic_cast<NotifiableWaitingSet*>(node.stage);
                      waitingset != nullptr) {
                for(const auto& point : waitingset->Slots()) {
                    if(!geometry->InsideGeometry(point)) {
                        faultyStages.push_back(stageId);
                    }
                }
            }
        }
    }

    if(!faultyAgents.empty() || !faultyStages.empty()) {
        std::string message = "Could not switch the geometry.\n";

        if(!faultyAgents.empty()) {
            message += fmt::format(
                "The following agents would be outside of the new geometry: {}\n",
                fmt::join(faultyAgents, ", "));
        }
        if(!faultyStages.empty()) {
            message += fmt::format(
                "The following stages would be outside of the new geometry: {}",
                fmt::join(faultyStages, ", "));
        }

        throw GeometrySwitchError(message.c_str(), faultyAgents, faultyStages);
    }
}

void Simulation::PushTimer(const std::string_view name, size_t probe_log_level)
{
    _timer.pushTimerProbe(name, probe_log_level);
}

void Simulation::PopTimer(const std::string_view name)
{
    _timer.popTimerProbe(name);
}

TimerEntry::duration_type Simulation::GetTimerDuration(const std::string_view name) const
{
    return _timer.getDuration(name);
}

std::map<std::string, TimerEntry::duration_type> Simulation::GetTimerDurations() const
{
    return _timer.getDurations();
}