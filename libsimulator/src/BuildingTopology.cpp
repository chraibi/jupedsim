// SPDX-License-Identifier: LGPL-3.0-or-later
#include "BuildingTopology.hpp"

void BuildingTopology::AddLanding(
    CollisionGeometry::ID from,
    const std::vector<Point>& polyFromPoints,
    CollisionGeometry::ID to,
    const std::vector<Point>& polyToPoints)
{
    _landings.push_back(
        {from,
         to,
         Polygon{polyFromPoints},
         Polygon{polyToPoints},
         polyFromPoints,
         polyToPoints});
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
