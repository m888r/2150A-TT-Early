#include "motion/xdrive.hpp"
#include "Eigen/Core"
#include "Eigen/LU"
#include "Eigen/MatrixFunctions"

namespace motion {
XDrive::XDrive(okapi::Motor &leftFront, okapi::Motor &rightFront,
               okapi::Motor &leftBack, okapi::Motor &rightBack,
               okapi::QSpeed vMax, double b) {}
}  // namespace motion