// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "WarpDriverModel.hpp"

#include <cstdint>

class WarpDriverModelBuilder
{
    double _timeHorizon;
    double _stepSize;
    double _sigma;
    double _timeUncertainty;
    double _velocityUncertainty;
    int _numSamples;
    double _jamSpeedThreshold;
    int _jamStepCount;
    uint64_t _rngSeed;

public:
    WarpDriverModelBuilder(
        double timeHorizon = 2.0,
        double stepSize = 0.5,
        double sigma = 0.3,
        double timeUncertainty = 0.5,
        double velocityUncertainty = 0.2,
        int numSamples = 20,
        double jamSpeedThreshold = 0.1,
        int jamStepCount = 10,
        uint64_t rngSeed = 42);
    WarpDriverModel Build();
};
