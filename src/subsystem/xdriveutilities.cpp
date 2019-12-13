#include "subsystem/xdriveutilities.hpp"

namespace subsystem {
namespace drive {

bool atTarget;
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

    double targetAngle = targetPose.heading.convert(okapi::radian);
    double targetAngleNorm =
        std::atan2(std::sin(targetAngle), std::cos(targetAngle));

    double currentAngle = currPose.heading.convert(okapi::radian);
    double currentAngleNorm =
        std::atan2(std::sin(targetAngle), std::cos(targetAngle));

    double error = targetAngleNorm - currentAngleNorm;
    error = std::atan2(std::sin(error), std::cos(error));

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
    Eigen::Vector3d u(power * std::cos(currentAngle),
                      power * std::sin(currentAngle), angularVelocity);

    driveKinematics.moveGlobal(x, u);

    if (fabs(error) <= turnSettled.convert(okapi::radian) &&
        linearDistance <= distanceSettled.convert(okapi::meter)) {
      stop();
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