#include "motion/pid.hpp"

#include "api.h"

#include <algorithm>
#include <cmath>

namespace motion {
PID::PID(double kP, double kI, double kD, double kBias)
    : tunings({ kP, kI, kD, kBias }) {}

double PID::calculate(double pV) {
    double error = setpoint - pV;
    double derivative = lastPV - pV;

    auto now = pros::millis();
    auto dT = now - lastCallTime;

    if (lastCallTime != 0) {
        derivative /= dT;
        if (maxErrorToIntegrate == -1 || std::abs(error) <= maxErrorToIntegrate) {
            integral += tunings.kI * error * dT;
        }
    } else {
        derivative = 0;
    }

    derivative = derivativeFilter->filter(derivative);

    if (shouldIntegralReset && std::copysign(1.0, error) != std::copysign(1.0, lastError)) {
        integral = 0;
    }

    integral = std::clamp(integral, minIntegral, maxIntegral);

    double command = tunings.kP * error + integral + tunings.kD * derivative + std::copysign(tunings.kBias, error);

    lastCallTime = now;
    lastError = error;
    lastPV = pV;

    return command;
}

void PID::reset() {
    integral = 0;
    lastCallTime = 0;
    lastPV = 0;
    lastError = 0;
}

double PID::getError() {
    return lastError;
}

void PID::setTarget(double target) {
    setpoint = target;
}

void PID::setMaxErrorToIntegrate(double maxError) {
    maxErrorToIntegrate = maxError;
}

void PID::setIntegralLimits(double min, double max) {
    minIntegral = min;
    maxIntegral = max;
}

void PID::setIntegralLimits() {
    if (tunings.kI != 0)
        setIntegralLimits(-1 / tunings.kI, 1 / tunings.kI);
}

void PID::setIntegralReset(bool shouldReset) {
    shouldIntegralReset = shouldReset;
}

void PID::setDerivativeFilter(std::unique_ptr<okapi::Filter> filter) {
    derivativeFilter.swap(filter);
}
}