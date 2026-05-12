// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "CollisionGeometry.hpp"
#include "Point.hpp"
#include "Polygon.hpp"

#include <vector>

// Captures the connectivity between per-level CollisionGeometries. A
// Landing is an oriented portal: an agent on level `from`, whose position
// lies inside `polyFrom`, can be transferred to level `to` (its (x,y)
// stays put, the landings overlap in plan). Used by the level-switch
// system to detect crossings each iteration.
class BuildingTopology
{
public:
    struct Landing {
        CollisionGeometry::ID from;
        CollisionGeometry::ID to;
        Polygon polyFrom;
        Polygon polyTo;
        // Raw vertex lists, kept alongside the Polygons for callers that
        // need to serialize landings (e.g. the SQLite writer for 3D viz).
        std::vector<Point> polyFromPoints;
        std::vector<Point> polyToPoints;
    };

    void AddLanding(
        CollisionGeometry::ID from,
        const std::vector<Point>& polyFromPoints,
        CollisionGeometry::ID to,
        const std::vector<Point>& polyToPoints);

    // First landing on `level` whose polyFrom contains `pos`, or nullptr.
    const Landing* FindLanding(CollisionGeometry::ID level, Point pos) const;

    const std::vector<Landing>& Landings() const { return _landings; }

private:
    std::vector<Landing> _landings;
};
