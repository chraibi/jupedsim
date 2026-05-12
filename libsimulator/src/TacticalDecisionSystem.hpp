// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "RoutingEngine.hpp"

class TacticalDecisionSystem
{
public:
    TacticalDecisionSystem() = default;
    ~TacticalDecisionSystem() = default;
    TacticalDecisionSystem(const TacticalDecisionSystem& other) = delete;
    TacticalDecisionSystem& operator=(const TacticalDecisionSystem& other) = delete;
    TacticalDecisionSystem(TacticalDecisionSystem&& other) = delete;
    TacticalDecisionSystem& operator=(TacticalDecisionSystem&& other) = delete;

    // `engineFor` resolves an agent's currentLevel to the RoutingEngine for
    // that level. Single-level callers pass a closure that returns the same
    // engine regardless of input.
    template <typename Agents, typename EngineFor>
    void Run(EngineFor&& engineFor, Agents&& agents) const
    {
        for(auto& agent : agents) {
            auto& routingEngine = engineFor(agent.currentLevel);
            agent.destination = routingEngine.ComputeWaypoint(agent.pos, agent.target);
        }
    }
};
