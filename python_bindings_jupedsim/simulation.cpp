// SPDX-License-Identifier: LGPL-3.0-or-later
#include "Simulation.hpp"

#include "CollisionGeometry.hpp"
#include "Journey.hpp"
#include "OperationalModel.hpp"
#include "Polygon.hpp"
#include "Stage.hpp"
#include "StageDescription.hpp"
#include "conversion.hpp"

#include <pybind11/attr.h>
#include <pybind11/cast.h>
#include <pybind11/detail/common.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h> // IWYU pragma: keep

#include <cstdint>
#include <map>
#include <memory>
#include <stdexcept>
#include <tuple>
#include <vector>

namespace py = pybind11;

void init_simulation(py::module_& m)
{
    py::class_<Simulation>(m, "Simulation")
        .def(
            py::init([](const OperationalModel* model, CollisionGeometry geometry, double dT) {
                if(!model) {
                    throw std::invalid_argument("model must not be None");
                }
                return std::make_unique<Simulation>(
                    model->Clone(), std::make_unique<CollisionGeometry>(geometry), dT);
            }),
            py::kw_only(),
            py::arg("model"),
            py::arg("geometry"),
            py::arg("dt"))
        .def(
            "add_waypoint_stage",
            [](Simulation& sim, std::tuple<double, double> position, double distance) {
                return sim.AddStage(WaypointDescription{intoPoint(position), distance}).getID();
            })
        .def(
            "add_queue_stage",
            [](Simulation& sim, const std::vector<std::tuple<double, double>>& positions) {
                return sim.AddStage(NotifiableQueueDescription{intoPoints(positions)}).getID();
            })
        .def(
            "add_waiting_set_stage",
            [](Simulation& sim, const std::vector<std::tuple<double, double>>& positions) {
                return sim.AddStage(NotifiableWaitingSetDescription{intoPoints(positions)}).getID();
            })
        .def(
            "add_exit_stage",
            [](Simulation& sim, const std::vector<std::tuple<double, double>>& polygon) {
                return sim.AddStage(ExitDescription{Polygon{intoPoints(polygon)}}).getID();
            })
        .def(
            "add_direct_steering_stage",
            [](Simulation& sim) { return sim.AddStage(DirectSteeringDescription{}).getID(); })
        .def(
            "add_journey",
            [](Simulation& sim, std::map<uint64_t, TransitionDescription>& journey) {
                auto native_journey = std::map<BaseStage::ID, TransitionDescription>{};
                for(const auto& [stage_id, desc] : journey) {
                    native_journey.emplace(stage_id, desc);
                }

                return sim.AddJourney(native_journey).getID();
            })
        .def(
            "add_agent",
            [](Simulation& sim, GenericAgent& agent) { return sim.AddAgent(agent).getID(); })
        .def(
            "mark_agent_for_removal",
            [](Simulation& sim, uint64_t id) { sim.MarkAgentForRemoval(id); })
        .def(
            "removed_agents",
            [](const Simulation& sim) {
                auto removed_agent_ids = sim.RemovedAgents();
                auto agent_ids = std::vector<GenericAgent::ID::underlying_type>();
                agent_ids.reserve(removed_agent_ids.size());
                for(auto agent_id : removed_agent_ids) {
                    agent_ids.emplace_back(agent_id.getID());
                }
                return agent_ids;
            })
        .def("iterate", [](Simulation& sim) { sim.Iterate(); })
        .def(
            "switch_agent_journey",
            [](Simulation& sim, uint64_t agentId, uint64_t journeyId, uint64_t stageId) {
                sim.SwitchAgentJourney(agentId, journeyId, stageId);
            },
            py::kw_only(),
            py::arg("agent_id"),
            py::arg("journey_id"),
            py::arg("stage_id"))
        .def("agent_count", [](const Simulation& sim) { return sim.AgentCount(); })
        .def("elapsed_time", [](const Simulation& sim) { return sim.ElapsedTime(); })
        .def("delta_time", [](const Simulation& sim) { return sim.DT(); })
        .def("iteration_count", [](const Simulation& sim) { return sim.Iteration(); })
        .def(
            "agents",
            [](Simulation& sim) { return py::make_iterator(sim.Agents()); },
            py::keep_alive<0, 1>())
        .def(
            "agent",
            [](Simulation& sim, uint64_t agentId) -> auto& { return sim.Agent(agentId); },
            py::arg("agent_id"),
            py::return_value_policy::reference)
        .def(
            "agents_in_range",
            [](Simulation& sim, std::tuple<double, double> pos, double distance) {
                auto agents_in_range = sim.AgentsInRange(intoPoint(pos), distance);
                auto agents = std::vector<uint64_t>();
                agents.reserve(agents_in_range.size());
                for(auto agent : agents_in_range) {
                    agents.emplace_back(agent.getID());
                }
                return agents;
            })
        .def(
            "agents_in_polygon",
            [](Simulation& sim, const std::vector<std::tuple<double, double>>& poly) {
                auto agents_in_range = sim.AgentsInPolygon(intoPoints(poly));
                auto agents = std::vector<uint64_t>();
                agents.reserve(agents_in_range.size());
                for(auto agent : agents_in_range) {
                    agents.emplace_back(agent.getID());
                }
                return agents;
            })
        .def("get_stage_proxy", [](Simulation& sim, uint64_t id) { return sim.Stage(id); })
        .def("set_tracing", [](Simulation& sim, bool status) { sim.SetTracing(status); })
        .def(
            "set_timer_log_level",
            [](Simulation& sim, size_t level) { sim.SetTimerLogLevel(level); })
        .def("get_geometry", [](Simulation& sim) { return sim.Geo(); })
        .def(
            "primary_level",
            [](const Simulation& sim) { return sim.PrimaryLevel().getID(); })
        .def(
            "push_timer",
            [](Simulation& sim, const std::string& name, size_t probe_log_level) {
                sim.PushTimer(name, probe_log_level);
            })
        .def("pop_timer", [](Simulation& sim, const std::string& name) { sim.PopTimer(name); })
        .def(
            "get_duration",
            [](Simulation& sim, const std::string_view name) { return sim.GetTimerDuration(name); })
        .def("get_durations", [](Simulation& sim) { return sim.GetTimerDurations(); })
        .def("switch_geometry", [](Simulation& sim, CollisionGeometry& geometry) {
            sim.SwitchGeometry(std::make_unique<CollisionGeometry>(geometry));
        })
        .def(
            "add_level",
            [](Simulation& sim, CollisionGeometry& geometry, double elevation) {
                return sim.AddLevel(std::make_unique<CollisionGeometry>(geometry), elevation)
                    .getID();
            },
            py::arg("geometry"),
            py::arg("elevation") = 0.0)
        .def(
            "level_ids",
            [](const Simulation& sim) {
                std::vector<uint64_t> ids;
                for(const auto id : sim.LevelIds()) {
                    ids.push_back(id.getID());
                }
                return ids;
            })
        .def(
            "level_elevation",
            [](const Simulation& sim, uint64_t id) { return sim.LevelElevation(id); },
            py::arg("level_id"))
        .def(
            "level_geometry",
            [](const Simulation& sim, uint64_t id) -> CollisionGeometry {
                return sim.LevelGeometry(id);
            },
            py::arg("level_id"))
        .def(
            "landings",
            [](const Simulation& sim) {
                std::vector<std::tuple<
                    uint64_t,
                    uint64_t,
                    std::vector<std::tuple<double, double>>,
                    std::vector<std::tuple<double, double>>>>
                    out;
                for(const auto& l : sim.Topology().Landings()) {
                    std::vector<std::tuple<double, double>> pf, pt;
                    for(const auto& p : l.polyFromPoints) {
                        pf.emplace_back(p.x, p.y);
                    }
                    for(const auto& p : l.polyToPoints) {
                        pt.emplace_back(p.x, p.y);
                    }
                    out.emplace_back(l.from.getID(), l.to.getID(), std::move(pf), std::move(pt));
                }
                return out;
            })
        .def(
            "add_landing",
            [](Simulation& sim,
               uint64_t fromLevel,
               const std::vector<std::tuple<double, double>>& polyFrom,
               uint64_t toLevel,
               const std::vector<std::tuple<double, double>>& polyTo) {
                sim.AddLanding(fromLevel, intoPoints(polyFrom), toLevel, intoPoints(polyTo));
            },
            py::kw_only(),
            py::arg("from_level"),
            py::arg("polygon_from"),
            py::arg("to_level"),
            py::arg("polygon_to"));
}
