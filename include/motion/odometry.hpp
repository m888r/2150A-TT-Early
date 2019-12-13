#pragma once

#include <atomic>
#include "api.h"
#include "okapi/api.hpp"
#include "structs.hpp"
#include "Eigen/Core"
#include "Eigen/MatrixFunctions"

namespace motion {
using namespace okapi::literals;
using namespace structs;
// struct Pose {
//   okapi::QLength x;
//   okapi::QLength y;
//   okapi::QAngle theta;

//   Pose() : x(0_in), y(0_in), theta(0_deg) {}

//   Pose(okapi::QLength x, okapi::QLength y, okapi::QAngle theta)
//       : x(x), y(y), theta(theta) {}
// };

class Odometry {
 public:
  Odometry(const okapi::ADIEncoder &leftEnc, const okapi::ADIEncoder &rightEnc,
           okapi::QLength chassisWidth, okapi::QLength wheelDiameter);
  Odometry(const okapi::ADIEncoder &leftEnc, const okapi::ADIEncoder &rightEnc,
           okapi::ADIEncoder &centerEnc, okapi::QLength chassisWidth,
           okapi::QLength wheelDiameter, okapi::QLength centerOffset);

  Pose getPose();
  void setPose(Pose desired);

  void update();

  void reset();

  okapi::QLength getDistanceTo(okapi::QLength x, okapi::QLength y);

  Eigen::Vector3d getStateVector();

 private:
  okapi::ADIEncoder leftEnc;
  okapi::ADIEncoder rightEnc;
  okapi::ADIEncoder *centerEnc; // TODO std::optional

  okapi::QLength chassisWidth;
  okapi::QLength wheelDiameter;
  okapi::QLength centerOffset = 0_in;


  pros::Mutex poseMutex;

  Pose currPose;
  Pose lastPose;

  pros::Task odomTask;

  double lastLeftEnc = 0;
  double lastRightEnc = 0;
  double lastMiddleEnc = 0;

  static void trampoline(void *instance);
  void run();

  double ticksToMeter(int ticks);
};
}  // namespace motion