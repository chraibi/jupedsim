// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "Point.hpp"

struct WarpDriverModelUpdate {
    Point position{};
    Point orientation{};
    int jamCounter{0};
    double stuckTime{0.0};
    double anchorX{0.0};
    double anchorY{0.0};
    double detourTime{0.0};
    int detourSide{1};
};
