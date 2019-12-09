#include "motion/odometry.hpp"

namespace motion {
using namespace structs;

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

  double deltaHeading = (dEncR - dEncL) / chassisWidth.convert(okapi::meter);

  double r1 = deltaHeading == 0 ? 0 : avgEnc / deltaHeading;

  PositionVector positionUpdate(
      okapi::meter *
          (deltaHeading == 0 ? avgEnc : (r1 * std::sin(deltaHeading))),
      okapi::meter *
          (deltaHeading == 0 ? 0 : (r1 - r1 * std::cos(deltaHeading))));

  double encC = (centerEnc != nullptr) ? ticksToMeter(centerEnc->get()) : 0;
  double centerWidth = centerOffset.convert(okapi::meter);

  double rC = deltaHeading == 0 ? 0 : encC / deltaHeading;

  PositionVector centerPositionUpdate(
      okapi::meter *
          (deltaHeading == 0 ? 0 : (rC - rC * std::cos(deltaHeading))),
      okapi::meter *
          (deltaHeading == 0 ? encC : (rC * std::sin(deltaHeading))));
  
  centerPositionUpdate.rotateSelf(storedPose.heading + (1_pi/2.0) * okapi::radian); 
  // remember to add vector from the center encoder to the center of the robot
  PositionVector centerOffsetVector(std::cos(deltaHeading) * centerOffset, std::sin(deltaHeading) * centerOffset);

  storedPose.position.addSelf(centerPositionUpdate);

  positionUpdate.rotateSelf(storedPose.heading);
  positionUpdate.setSelf(PositionVector(positionUpdate.getX(),  // 😂
                                        -positionUpdate.getY()));
  storedPose.position.addSelf(positionUpdate);
  storedPose.turn(okapi::radian * deltaHeading);

  while (storedPose.heading >= (2_pi) * okapi::radian) {
    storedPose.heading -= 2_pi * okapi::radian;
  }
  while (storedPose.heading < 0_rad) {
    storedPose.heading += 2_pi * okapi::radian;
  }

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