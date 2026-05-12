// SPDX-License-Identifier: LGPL-3.0-or-later
#include "BuildingTopology.hpp"

void BuildingTopology::AddLanding(
    CollisionGeometry::ID from,
    Polygon polyFrom,
    CollisionGeometry::ID to,
    Polygon polyTo)
{
    _landings.push_back({from, to, std::move(polyFrom), std::move(polyTo)});
}

const BuildingTopology::Landing*
BuildingTopology::FindLanding(CollisionGeometry::ID level, Point pos) const
{
    for(const auto& l : _landings) {
        if(l.from == level && l.polyFrom.IsInside(pos)) {
            return &l;
        }
    }
    return nullptr;
}
