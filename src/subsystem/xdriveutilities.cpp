#include "subsystem/xdriveutilities.hpp"

namespace subsystem {
namespace drive {

PIDGains defaultStraightGains(4.05, 0, 0);  // vel 2.3, volt: was 3.8
PIDGains defaultTurnGains(1.0, 0, 0);       // vel 0.6
// maximum omega possible is (3.25" * 0.0254 * pi * (200/60)) / (chassisWidth /
// 2)
okapi::QAngularSpeed defaultOmega =
    0.8 * okapi::radps;  // 0.128 maxfor 13.5"? < nah check config.cpp
okapi::QAngle defaultPIDThreshold = 12_deg;
bool enabled = false;
bool atTarget = false;
okapi::QAngle defaultTurnSettled = 2_deg;         // 3 degrees
okapi::QLength defaultDistanceSettled = 1.18_in;  // 5 cm
// max velocity if the robot was going straight forward in m/s
double defaultMaxLinearVelocity = 1000000.0;

Pose currentTargetPose(0_in, 0_in, 0_deg);

void moveTo(Pose targetPose, std::optional<okapi::QLength> straightSettle,
            std::optional<okapi::QAngle> turnSettle,
            std::optional<okapi::QAngularSpeed> angularVelocityCap,
            std::optional<okapi::QAngle> defaultPIDTthreshold,
            std::optional<PIDGains> straightGains,
            std::optional<PIDGains> turnGains,
            std::optional<double> maxLinearVelocity,
            std::optional<double> velSettle) {
  using namespace motion;
  using namespace robot;
  using namespace structs;

  PIDGains straight = straightGains.value_or(defaultStraightGains);
  PIDGains turn = turnGains.value_or(defaultTurnGains);
  double w = angularVelocityCap.value_or(defaultOmega).convert(okapi::radps);
  double errorToPID = defaultPIDThreshold.convert(okapi::radian);

  double maxLinVel = maxLinearVelocity.value_or(defaultMaxLinearVelocity);
  double velSettled = velSettle.value_or(defaultMaxLinearVelocity);

  okapi::QLength distanceSettled =
      straightSettle.value_or(defaultDistanceSettled);
  okapi::QAngle turnSettled = turnSettle.value_or(defaultTurnSettled);

  PID straightPID(straight.kP, straight.kI, straight.kD);
  PID xPID(straight.kP, straight.kI, straight.kD);
  PID yPID(straight.kP, straight.kI, straight.kD);
  PID turnPID(turn.kP, turn.kI, turn.kD);

  PositionVector targetPosition = targetPose.position;
  currentTargetPose = targetPose;

  straightPID.setTarget(0);
  turnPID.setTarget(0);

  enabled = true;
  atTarget = false;

  okapi::Timer printTimer;

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

    xPower = std::clamp(xPower, -maxLinVel, maxLinVel);
    yPower = std::clamp(yPower, -maxLinVel, maxLinVel);

    double targetAngle = targetPose.heading.convert(okapi::radian);
    double targetAngleNorm =
        std::atan2(std::sin(targetAngle), std::cos(targetAngle));

    double currentAngle = currPose.heading.convert(okapi::radian);
    double currentAngleNorm =
        std::atan2(std::sin(currentAngle), std::cos(currentAngle));

    double error = targetAngleNorm - currentAngleNorm;
    error = std::atan2(std::sin(error), std::cos(error));
    // printf("Error: %1.2f, ", error);

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

    // printf("Error: %1.2f, target: %1.2f, current: %1.2f", error,
    // targetAngleNorm, currentAngleNorm);

    Eigen::Vector4d uState(robot::frontLeft.getActualVelocity(),
                           robot::frontRight.getActualVelocity(),
                           robot::backRight.getActualVelocity(),
                           robot::backLeft.getActualVelocity());

    uState = driveKinematics.angularWheelSpeedToLinear(3.25_in, uState);

    auto xDot = driveKinematics.fk(x, uState);

    auto currentLinearVelocity =
        std::sqrt((xDot(0) * xDot(0)) + (xDot(1) * xDot(1)));

    double currentLinearPower =
        std::sqrt(std::pow(u(0), 2) + std::pow(u(1), 2));

    if (printTimer.repeat(100_ms)) {
      printf(
          "Linear Power: %1.2f, Lin Vel: %1.2f, Angular Vel: %1.2f, Ang Error: "
          "%1.2f\n",
          currentLinearPower, currentLinearVelocity, angularVelocity, error);
    }
    driveKinematics.moveGlobalVoltage(x, u);

    if (fabs(error) <= turnSettled.convert(okapi::radian) &&
        linearDistance <= distanceSettled.convert(okapi::meter) &&
        currentLinearVelocity <= velSettled) {
      printf("broke out, vel settled: %1.2f, curr vel: %1.2f", velSettled,
             currentLinearVelocity);
      // printf("IS AT TARGET RN");
      atTarget = true;
    } else {
      atTarget = false;
    }

    pros::delay(10);
  }

  // driveKinematics.stop();
}

void stop() {
  enabled = false;
  atTarget = false;
  robot::driveKinematics.stop();
}

bool isAtTarget() { return atTarget; }

void waitUntilSettled() {
  while (!drive::isAtTarget()) {
    // printf("Still Waiting\n");
    pros::delay(10);
  }
  drive::stop();
}

void wusNoStop() {
  while (!drive::isAtTarget()) {
    pros::delay(10);
  }
  enabled = false;
  atTarget = false;
}

okapi::QLength getDistanceToTarget() {
  auto target = currentTargetPose.position;
  auto current = robot::odometry.getPose().position;
  auto difference = PositionVector::subtract(target, current);

  return std::sqrt((difference.getX().convert(okapi::inch) *
                    difference.getX().convert(okapi::inch)) +
                   (difference.getY().convert(okapi::inch) *
                    difference.getY().convert(okapi::inch))) *
         okapi::inch;
}

double getLinearVelocity() {
  auto currPose = robot::odometry.getPose();
  auto currentAngle = currPose.heading.convert(okapi::radian);

  Eigen::Vector3d x(currPose.position.getX().convert(okapi::meter),
                    currPose.position.getY().convert(okapi::meter),
                    currentAngle);

  Eigen::Vector4d uState(robot::frontLeft.getActualVelocity(),
                         robot::frontRight.getActualVelocity(),
                         robot::backRight.getActualVelocity(),
                         robot::backLeft.getActualVelocity());

  uState = driveKinematics.angularWheelSpeedToLinear(3.25_in, uState);

  auto xDot = driveKinematics.fk(x, uState);

  auto currentLinearVelocity =
      std::sqrt((xDot(0) * xDot(0)) + (xDot(1) * xDot(1)));
  
  return currentLinearVelocity;
}

void printXDriveData() {
  while (true) {
    auto pose = odometry.getPose();

    Eigen::Vector3d x(pose.position.getX().convert(okapi::foot),
                      pose.position.getY().convert(okapi::foot),
                      pose.heading.convert(okapi::radian));

    Eigen::Vector4d u(robot::frontLeft.getActualVelocity(),
                      robot::frontRight.getActualVelocity(),
                      robot::backRight.getActualVelocity(),
                      robot::backLeft.getActualVelocity());

    u = driveKinematics.angularWheelSpeedToLinear(3.25_in, u);

    auto xDot = driveKinematics.fk(x, u);

    printf("%1.2f,%1.2f,%1.2f,%1.2f,%1.2f,%1.2f\n", x(0), x(1), x(2), xDot(0),
           xDot(1), xDot(2));

    pros::delay(50);
  }
}

void initXDriveDebug() {
  pros::Task printXDriveDataTask(printXDriveData, "Print X Drive Data");
}
}  // namespace drive
}  // namespace subsystem