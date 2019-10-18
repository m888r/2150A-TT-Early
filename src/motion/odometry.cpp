#include "motion/odometry.hpp"

namespace motion {
Odometry::Odometry(const okapi::ADIEncoder &leftEnc,
                   const okapi::ADIEncoder &rightEnc,
                   okapi::QLength chassisWidth, okapi::QLength wheelDiameter)
    : leftEnc(std::move(leftEnc)),
      rightEnc(std::move(rightEnc)),
      chassisWidth(chassisWidth),
      wheelDiameter(wheelDiameter),
      odomTask(trampoline, this, "Odometry"),
      currPose(0_in, 0_in, 0_deg),
      lastPose(currPose) {
  centerEnc = nullptr;
}
Odometry::Odometry(const okapi::ADIEncoder &leftEnc,
                   const okapi::ADIEncoder &rightEnc,
                   okapi::ADIEncoder &centerEnc, okapi::QLength chassisWidth,
                   okapi::QLength wheelDiameter, okapi::QLength centerOffset)
    : leftEnc(std::move(leftEnc)),
      rightEnc(std::move(rightEnc)),
      centerEnc(&centerEnc),
      chassisWidth(chassisWidth),
      wheelDiameter(wheelDiameter),
      centerOffset(centerOffset),
      odomTask(trampoline, this, "Odometry"),
      currPose(0_in, 0_in, 0_deg),
      lastPose(currPose) {}

Pose Odometry::getPose() {
  poseMutex.take(TIMEOUT_MAX);
  Pose returnPose = currPose;
  poseMutex.give();
  return returnPose;
}

void Odometry::setPose(Pose desired) {
  poseMutex.take(TIMEOUT_MAX);
  lastPose = desired;
  currPose = desired;
  poseMutex.give();
}

double Odometry::ticksToMeter(int ticks) {
  return (ticks / 360.0) * (1_pi * wheelDiameter.convert(okapi::meter));
}

void Odometry::update() {
  poseMutex.take(TIMEOUT_MAX);
  Pose storedPose = currPose;
  poseMutex.give();

  double encL = ticksToMeter(leftEnc.get());
  double encR = ticksToMeter(rightEnc.get());
  double dEncL = encL - lastLeftEnc;
  double dEncR = encR - lastRightEnc;
  double avgEnc = (dEncL + dEncR) / 2.0;

  double deltaHeading = (dEncL - dEncR) / chassisWidth.convert(okapi::meter);
  storedPose.theta += deltaHeading * okapi::radian;

  lastLeftEnc = encL;
  lastRightEnc = encR;
  
  poseMutex.take(TIMEOUT_MAX);
  currPose = storedPose;
  poseMutex.give();
}

void Odometry::run() {
  while (true) {
    update();
    pros::delay(10);
  }
}

void Odometry::trampoline(void *instance) {
  static_cast<Odometry *>(instance)->run();
}

}  // namespace motion