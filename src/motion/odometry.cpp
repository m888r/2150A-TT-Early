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
  double dEncC = encC - lastMiddleEnc;
  double centerWidth = centerOffset.convert(okapi::meter);

  double rC = deltaHeading == 0 ? 0 : dEncC / deltaHeading;

  PositionVector centerPositionUpdate(
      okapi::meter *
          (deltaHeading == 0 ? 0 : (rC - rC * std::cos(deltaHeading))),
      okapi::meter *
          (deltaHeading == 0 ? dEncC : (rC * std::sin(deltaHeading))));
  
  centerPositionUpdate.rotateSelf(storedPose.heading/* + 90_deg*/); 
  // remember to add vector from the center encoder to the center of the robot but without constantly adding when the robot isn't moving
  //PositionVector centerOffsetVector(std::cos(deltaHeading) * centerOffset, std::sin(deltaHeading) * centerOffset);
  //PositionVector centerOffsetVector(0_in, centerOffset);
  // if (deltaHeading != 0) {
  //   positionUpdate.addSelf(centerOffsetVector);
  // }
  PositionVector centerOffsetVector(centerOffset, 0_in);
  centerPositionUpdate.subtractSelf(centerOffsetVector);
  centerPositionUpdate.setSelf(PositionVector(positionUpdate.getX(),  // 😂
                                        -positionUpdate.getY()));
  centerOffsetVector.rotateSelf(deltaHeading * okapi::radian);
  centerPositionUpdate.addSelf(centerOffsetVector);

  positionUpdate.addSelf(centerPositionUpdate);

  positionUpdate.rotateSelf(storedPose.heading);
  positionUpdate.setSelf(PositionVector(positionUpdate.getX(),  // 😂
                                        -positionUpdate.getY()));
  storedPose.position.addSelf(positionUpdate);
  storedPose.turn(okapi::radian * deltaHeading);

  while (storedPose.heading >= 360_deg) {
    storedPose.heading -= 360_deg;
  }
  while (storedPose.heading < 0_rad) {
    storedPose.heading += 360_deg;
  }

  lastLeftEnc = encL;
  lastRightEnc = encR;
  lastMiddleEnc = encC;

  poseMutex.take(TIMEOUT_MAX);
  currPose = storedPose;
  poseMutex.give();
}

okapi::QLength Odometry::getDistanceTo(okapi::QLength x, okapi::QLength y) {
  Pose storedPose;
  poseMutex.take(TIMEOUT_MAX);
  storedPose = currPose;
  poseMutex.give();

  double x2 = x.convert(okapi::meter);
  double y2 = y.convert(okapi::meter);
  double x1 = storedPose.position.getX().convert(okapi::meter);
  double y1 = storedPose.position.getY().convert(okapi::meter);
  
  return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2)) * okapi::meter;
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