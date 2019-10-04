#include "motion/profiling.hpp"

namespace motion {
ProfileOutput::ProfileOutput(double position, double velocity, double accel)
    : position(position)
    , velocity(velocity)
    , accel(accel) {}
}