#pragma once

#include "okapi/api/filter/passthroughFilter.hpp"

#include <memory>

namespace motion {
struct PIDTunings {
    double kP;
    double kI;
    double kD;
    double kBias;
};

class PID {
    PIDTunings tunings;
    std::unique_ptr<okapi::Filter> derivativeFilter{ std::make_unique<okapi::PassthroughFilter>() };

    double setpoint = 0;
    double integral = 0;
    double lastPV = 0;
    double lastError = 0;
    unsigned long lastCallTime = 0;
    double maxErrorToIntegrate = -1;
    double minIntegral = -1;
    double maxIntegral = 1;
    bool shouldIntegralReset = true;

public:
    PID(double kP, double kI, double kD, double kBias = 0.0);

    double calculate(double pV);
    double getError();
    void reset();
    void setTarget(double target);
    void setMaxErrorToIntegrate(double maxError);
    void setIntegralLimits(double min, double max);
    void setDerivativeFilter(std::unique_ptr<okapi::Filter> filter);
    void setIntegralLimits();
    void setIntegralReset(bool shouldReset);
};
}