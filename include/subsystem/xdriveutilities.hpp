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

void moveTo(Pose targetPose, std::optional<okapi::QLength> straightSettle = std::nullopt, std::optional<okapi::QAngle> turnSettle = std::nullopt, std::optional<okapi::QAngularSpeed> omegaDesired = std::nullopt,
            std::optional<okapi::QAngle> defaultPIDTthreshold = std::nullopt,
            std::optional<PIDGains> straightGains = std::nullopt,
            std::optional<PIDGains> turnGains = std::nullopt);

void stop();

bool isAtTarget();

void waitUntilSettled();
}  // namespace drive
}  // namespace subsystem