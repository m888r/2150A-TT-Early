#pragma once

#include "motion/odometry.hpp"
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

extern okapi::ADIEncoder rightEnc;
extern okapi::ADIEncoder leftEnc;
extern okapi::ADIEncoder centerEnc;

extern motion::Odometry odometry;

extern okapi::XDriveModel xDrive;

extern okapi::QLength wheelDiameter;
extern okapi::QLength encoderWheelDiameter;
extern okapi::QLength encoderWidth;

}  // namespace robot