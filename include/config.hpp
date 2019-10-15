#pragma once

#include "okapi/api.hpp"

namespace robot {
  extern okapi::Motor frontRight;
  extern okapi::Motor frontLeft;
  extern okapi::Motor backRight;
  extern okapi::Motor backLeft;

  extern okapi::Motor intakeRight;
  extern okapi::Motor intakeLeft;
  extern okapi::MotorGroup intakeGroup;
  
  extern okapi::Motor tilt;

  extern okapi::Motor lift;

  extern okapi::AsyncMotionProfileController straightAMPC;
}