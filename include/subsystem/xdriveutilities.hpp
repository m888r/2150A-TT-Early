#pragma once
#include <optional>
#include "Eigen/Core"
#include "Eigen/MatrixFunctions"
#include "drive.hpp"
#include "okapi/api.hpp"
#include "structs.hpp"

namespace subsystem {
namespace drive {
using namespace structs;
using namespace robot;

void moveTo(Pose targetPose,
            std::optional<okapi::QLength> straightSettle = std::nullopt,
            std::optional<okapi::QAngle> turnSettle = std::nullopt,
            std::optional<okapi::QAngularSpeed> omegaDesired = std::nullopt,
            std::optional<okapi::QAngle> defaultPIDTthreshold = std::nullopt,
            std::optional<PIDGains> straightGains = std::nullopt,
            std::optional<PIDGains> turnGains = std::nullopt, std::optional<double> maxLinearVelocity = std::nullopt);

void stop();

bool isAtTarget();

void waitUntilSettled();

void wusNoStop();

void printXDriveData();

void initXDriveDebug();
}  // namespace drive
}  // namespace subsystem