#pragma once

namespace motion {
struct ProfileOutput {
    double position;
    double velocity;
    double accel;

    ProfileOutput(double position = 0.0, double velocity = 0.0, double accel = 0.0);
};

class MotionProfile {
public:
    virtual ProfileOutput calculate(double t) = 0;
    virtual double getTotalTime() = 0;
};
}
