#pragma once

#include "config.hpp"
#include "motion/pid.hpp"
#include "okapi/api.hpp"

namespace subsystem {
namespace drive {

  void turnTo(okapi::QAngle angle);

  void moveDistanceProfile(double distance, bool backwards = false, double maxVel = 1.05, double maxAccel = 2.0);
  
  void waitUntilSettled();

}
}  // namespace motion