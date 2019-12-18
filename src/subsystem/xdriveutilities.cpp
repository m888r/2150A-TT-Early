#include "subsystem/xdriveutilities.hpp"

namespace subsystem {
namespace drive {

PIDGains defaultStraightGains(2.3, 0, 0);
PIDGains defaultTurnGains(0.6, 0, 0);
// maximum omega possible is (3.25" * 0.0254 * pi * (200/60)) / (chassisWidth /
// 2)
okapi::QAngularSpeed defaultOmega =
    0.8 * okapi::radps;  // 0.128 maxfor 13.5"? < nah check config.cpp
okapi::QAngle defaultPIDThreshold = 12_deg;
bool enabled = false;
bool atTarget = false;
okapi::QAngle turnSettled = 2_deg;      // 3 degrees
okapi::QLength distanceSettled = 3_cm;  // 5 cm

void moveTo(Pose targetPose,
            std::optional<okapi::QAngularSpeed> angularVelocityCap,
            std::optional<okapi::QAngle> defaultPIDTthreshold,
            std::optional<PIDGains> straightGains,
            std::optional<PIDGains> turnGains) {
  using namespace motion;
  using namespace robot;
  using namespace structs;

  PIDGains straight = straightGains.value_or(defaultStraightGains);
  PIDGains turn = turnGains.value_or(defaultTurnGains);
  double w = angularVelocityCap.value_or(defaultOmega).convert(okapi::radps);
  double errorToPID = defaultPIDThreshold.convert(okapi::radian);

  PID straightPID(straight.kP, straight.kI, straight.kD);
  PID xPID(straight.kP, straight.kI, straight.kD);
  PID yPID(straight.kP, straight.kI, straight.kD);
  PID turnPID(turn.kP, turn.kI, turn.kD);

  PositionVector targetPosition = targetPose.position;

  straightPID.setTarget(0);
  turnPID.setTarget(0);

  enabled = true;
  atTarget = false;

  while (enabled) {
    auto currPose = odometry.getPose();
    double linearDistance =
        odometry.getDistanceTo(targetPosition.getX(), targetPosition.getY())
            .convert(okapi::meter);
    double power = straightPID.calculate(linearDistance);
    xPID.setTarget(targetPosition.getX().convert(okapi::meter));
    yPID.setTarget(targetPosition.getY().convert(okapi::meter));

    double xPower =
        xPID.calculate(currPose.position.getX().convert(okapi::meter));
    double yPower =
        yPID.calculate(currPose.position.getY().convert(okapi::meter));

    double targetAngle = targetPose.heading.convert(okapi::radian);
    double targetAngleNorm =
        std::atan2(std::sin(targetAngle), std::cos(targetAngle));

    double currentAngle = currPose.heading.convert(okapi::radian);
    double currentAngleNorm =
        std::atan2(std::sin(currentAngle), std::cos(currentAngle));

    double error = targetAngleNorm - currentAngleNorm;
    error = std::atan2(std::sin(error), std::cos(error));
    //printf("Error: %1.2f, ", error);

    double angularVelocity;
    if (fabs(error) > errorToPID) {
      angularVelocity = w;
      angularVelocity *= (error != 0) ? (fabs(error) / error) : 0;
    } else {
      angularVelocity = turnPID.calculate(-error);
      angularVelocity = std::clamp(angularVelocity, -w, w);
    }

    Eigen::Vector3d x(currPose.position.getX().convert(okapi::meter),
                      currPose.position.getY().convert(okapi::meter),
                      currentAngle);
    Eigen::Vector3d u(-xPower, -yPower, angularVelocity);

    //printf("Error: %1.2f, target: %1.2f, current: %1.2f", error, targetAngleNorm, currentAngleNorm);
    printf("Linear Power: %1.2f, Angular Vel: %1.2f, Ang Error: %1.2f\n", error, power, angularVelocity, error);
    driveKinematics.moveGlobal(x, u);

    if (fabs(error) <= turnSettled.convert(okapi::radian) &&
        linearDistance <= distanceSettled.convert(okapi::meter)) {
          //printf("IS AT TARGET RN");
      atTarget = true;
    } else {
      atTarget = false;
    }

    pros::delay(10);
  }

  driveKinematics.stop();
}

void stop() {
  enabled = false;
  robot::driveKinematics.stop();
}

bool isAtTarget() { return atTarget; }
}  // namespace drive
}  // namespace subsystem