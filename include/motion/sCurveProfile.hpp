#pragma once

#include "API.h"
#include "structs.hpp"
#include "motion/profiling.hpp"

namespace motion {
class SCurveProfile : public MotionProfile {

    // runs initial calculations
    void setup();

    // helpers for some of the more tedious math used in setup
    void calculateAccelPeriodDisplacement();
    void calculateVelocityConstraint();

    structs::KinematicConstraints constraints;
    double totalDisplacement;

    double velocityConstraint();

    double accelPeriodDisplacement;
    double t1;

    double v1;
    double v2;

    double t2;
    double d1;
    double d2;

    double t3;

public:
    SCurveProfile(structs::KinematicConstraints constraints, double displacement);

    ProfileOutput calculate(double t) override;
    double getTotalTime() override;
};
}