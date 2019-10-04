#include "motion/sCurveProfile.hpp"

#include <cmath>

namespace motion {
SCurveProfile::SCurveProfile(structs::KinematicConstraints constraints, double displacement)
    : constraints(constraints)
    , totalDisplacement(displacement) {
    setup();
}

void SCurveProfile::calculateAccelPeriodDisplacement() {
    auto maxVel = constraints.maxVel;
    auto maxAccel = constraints.maxAccel;
    auto maxJerk = constraints.maxJerk;

    double timeToMaxAccel = maxAccel / maxJerk;
    double timeToZeroAccel = timeToMaxAccel;

    double velocityAtEndOfInitialJerk = (maxAccel * timeToMaxAccel) / 2;
    double velocityToMakeupWithAccelHold = maxVel - (2 * velocityAtEndOfInitialJerk);
    double timeToHoldMaxAccel = velocityToMakeupWithAccelHold / maxAccel;

    double d1 = maxJerk / 6 * pow(timeToMaxAccel, 3); // positive jerk
    double d2 = velocityAtEndOfInitialJerk * timeToHoldMaxAccel + maxAccel / 2 * pow(timeToHoldMaxAccel, 2); // no jerk
    double d3 = (velocityAtEndOfInitialJerk + velocityToMakeupWithAccelHold + maxJerk / 2 * pow(timeToMaxAccel, 2)) * timeToZeroAccel - maxJerk / 6 * pow(timeToZeroAccel, 3); // negative jerk

    accelPeriodDisplacement = d1 + d2 + d3;
}

void SCurveProfile::calculateVelocityConstraint() {
    auto maxAccel = constraints.maxAccel;
    auto maxJerk = constraints.maxJerk;

    double displacementPerAccelPeriod = totalDisplacement / 2;
    constraints.maxVel = (-pow(maxAccel, 2) + pow(fabs(pow(maxAccel, 4) + 8.0 * displacementPerAccelPeriod * maxAccel * pow(maxJerk, 2)), 0.5)) / (maxJerk * 2.0);
}

void SCurveProfile::setup() {
    calculateAccelPeriodDisplacement();

    if (totalDisplacement < accelPeriodDisplacement * 2) {
        calculateVelocityConstraint();
        calculateAccelPeriodDisplacement();
    }

    auto maxVel = constraints.maxVel;
    auto maxAccel = constraints.maxAccel;
    auto maxJerk = constraints.maxJerk;

    t1 = maxAccel / maxJerk;

    v1 = maxJerk / 2 * pow(t1, 2);
    v2 = maxVel - 2 * v1;

    t2 = v2 / maxAccel;

    d1 = accelPeriodDisplacement;
    d2 = totalDisplacement - 2 * d1;

    t3 = d2 / maxVel;
}

double SCurveProfile::getTotalTime() {
    return t1 * 4 + t2 * 2 + t3;
}

ProfileOutput SCurveProfile::calculate(double t) {
    auto maxVel = constraints.maxVel;
    auto maxAccel = constraints.maxAccel;
    auto maxJerk = constraints.maxJerk;

    if (t < t1) {
        return ProfileOutput(maxJerk / 2.0 * pow(t, 2.0), maxJerk * t);
    }

    t -= t1;

    if (t < t2) {
        return ProfileOutput(v1 + maxAccel * t, maxAccel);
    }

    t -= t2;

    if (t < t1) {
        return ProfileOutput(
            -maxJerk / 2 * pow(t1 - t, 2) + maxJerk / 2 * pow(t1, 2) + v1 + v2,
            maxJerk * (t1 - t));
    }

    t -= t1;

    if (t < t3) {
        return ProfileOutput(maxVel, 0);
    }

    t -= t3;

    if (t < t1) {
        return ProfileOutput(maxVel - maxJerk / 2 * pow(t, 2), -maxJerk * t);
    }

    t -= t1;

    if (t < t2) {
        return ProfileOutput(maxVel - v1 - maxAccel * t, -maxAccel);
    }

    t -= t2;

    if (t < t1) {
        return ProfileOutput(maxJerk / 2 * pow(t1 - t, 2), -maxJerk * (t1 - t));
    }

    return ProfileOutput(0, 0);
}
}