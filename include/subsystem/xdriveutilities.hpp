#pragma once
#include "Eigen/Core"
#include "Eigen/MatrixFunctions"
#include "drive.hpp"
#include "okapi/api.hpp"
#include "structs.hpp"
#include <optional>

namespace subsystem {
namespace drive {
using namespace structs;
using namespace robot;

PIDGains defaultStraightGains(0, 0, 0);
PIDGains defaultTurnGains(0, 0, 0);
// maximum omega possible is (3.25" * 0.0254 * pi * (200/60)) / (chassisWidth / 2)
okapi::QAngularSpeed defaultOmega = 0.04 * okapi::radps; // 0.128 maxfor 13.5"?
okapi::QAngle defaultPIDThreshold = 5_deg;
bool enabled = false;

void moveTo(Pose targetPose, std::optional<double> omegaDesired,
            std::optional<okapi::QAngle> defaultPIDTthreshold,
            std::optional<PIDGains> straightGains,
            std::optional<PIDGains> turnGains);

void stop();

bool isAtTarget();
}  // namespace drive
}  // namespace subsystem