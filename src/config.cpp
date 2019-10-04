#include "config.hpp"

#include "okapi/api.hpp"
#include "api.h"

namespace robot {
  okapi::Motor frontRight(-20);
  okapi::Motor frontLeft(19);
  okapi::Motor backRight(-10);
  okapi::Motor backLeft(3);

  okapi::Motor intakeRight(18);
  okapi::Motor intakeLeft(-1);
  
  okapi::Motor tilt(9);

  okapi::Motor lift(8);
}