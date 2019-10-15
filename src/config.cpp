#include "config.hpp"

#include "okapi/api.hpp"
#include "api.h"

namespace robot {
  okapi::Motor frontRight(-20);
  okapi::Motor frontLeft(10);
  okapi::Motor backRight(-11);
  okapi::Motor backLeft(1);

  okapi::Motor intakeRight(15);
  okapi::Motor intakeLeft(-7);
  okapi::MotorGroup intakeGroup({intakeRight, intakeLeft});
  
  okapi::Motor tilt(14);

  okapi::Motor lift(13);

  // okapi::ChassisController chassisStraight = okapi::ChassisControllerBuilder.withMotors({frontLeft, frontRight}, {backLeft, backRight}).withDimensions({3.25 * okapi::inch, 11 * okapi::inch}).build();

  // okapi::AsyncMotionProfileController straightAMPC = okapi::AsyncMotionProfileControllerBuilder.withOutput().buildMotionProfileController();
}