#pragma once

#include "config.hpp"
#include "motion/pid.hpp"
#include "okapi/api.hpp"

namespace subsystem {
namespace drive {

  void turnTo(okapi::QAngle angle);

  using namespace okapi::literals;

  void moveDistanceProfile(double distance, okapi::QAngle angleTarget = 0_deg, double maxVel = 1.22, double maxAccel = 1.0, bool strafing = false);
  
  void waitUntilSettled();

  bool isProfileDone();

}
}  // namespace motion