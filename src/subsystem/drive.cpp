#include "subsystem/drive.hpp"

namespace subsystem {
namespace drive {

bool profileDone = true;

bool isProfileDone() { return profileDone; }

void turnTo(okapi::QAngle angle) {
  // using namespace okapi::literals;

  // auto su = okapi::SettledUtilFactory::create(0.7, 0.4, 200_ms);
  // robot::xDrive.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);

  // auto angleRad = angle.convert(okapi::radian);

  // motion::PID pid(0.027, 0.0001, 1.5);  // 0.027 0.0003 1.6
  // pid.setMaxErrorToIntegrate(15);
  // pid.setIntegralReset(true);
  // double degreeTarget = std::atan2(std::sin(angle.convert(okapi::radian)),
  //                             std::cos(angle.convert(okapi::radian))) *
  //                       180.0 / 1_pi;
  // // while (angle >= 360_deg) {
  // //   angle -= 360_deg;
  // // }
  // // while (angle < 0_rad) {
  // //   angle += 360_deg;
  // // }

  // pid.setTarget(0);

  // double current = robot::odometry.getPose().heading.convert(okapi::degree);
  // current = std::atan2(std::sin(current), std::cos(current)) * 180.0 / 1_pi;

  // double error = degreeTarget - current;
  // error *= 1_pi / 180.0;
  // error = std::atan2(std::sin(error), std::cos(error));

  // while (!su.isSettled(/*pid.getError()*/error)) {
  //   robot::odometry.update();
  //   current = robot::odometry.getPose().heading.convert(okapi::degree);
  //   // current = atan2(sin(current), cos(current)) * 180.0 / 1_pi;
  //   error = (degreeTarget - current) * 1_pi / 180.0;
  //   error = std::atan2(std::sin(error), std::cos(error)) * 180.0 / 1_pi;

  //   double out = pid.calculate(error);
  //   //printf("Target: %1.2f, current: %1.2f, error: %1.2f, Power: %1.2f\n",
  //   //       degreeTarget, current, error, out);
  //   robot::xDrive.driveVectorVoltage(0, out);
  //   pros::delay(10);
  // }

  // robot::xDrive.stop();
}

void moveDriveLinearVelocity(double velLeft, double velRight,
                             bool strafing = false) {
  using namespace okapi::literals;

  double rpmLeft = velLeft /
                   (1_pi * robot::wheelDiameter.convert(okapi::meter)) *
                   (60.0 /*sec*/ / 1.0 /*min*/);
  double rpmRight = velRight /
                    (1_pi * robot::wheelDiameter.convert(okapi::meter)) *
                    (60.0 /*sec*/ / 1.0 /*min*/);
  // robot::xDrive.driveVector(rpm / 200.0, 0);
  if (!strafing) {
    robot::xDrive.right(rpmRight / 200.0);
    robot::xDrive.left(rpmLeft / 200.0);
  } else {  // rpmleft is front wheels
    robot::xDrive.getTopLeftMotor()->moveVelocity(rpmLeft);
    robot::xDrive.getTopRightMotor()->moveVelocity(-rpmLeft);
    robot::xDrive.getBottomLeftMotor()->moveVelocity(-rpmRight);
    robot::xDrive.getBottomRightMotor()->moveVelocity(rpmRight);
  }
}

void moveDistanceProfile(double distanceTarg, okapi::QAngle angleTarget,
                         double maxVel, double maxAccel, bool strafing) {
  profileDone = false;
  int direction = (distanceTarg < 0) ? -1 : 1;
  double distance = fabs(distanceTarg);

  motion::PID anglePID(0.08, 0.0, 0);

  double tAccel = maxVel / maxAccel;
  double dAccel = 1.0 / 2.0 * tAccel * maxVel;

  if (dAccel * 2.0 > distance) {
    double dAccelDesired = distance / 2.0;
    double newVmax = std::sqrt(dAccelDesired * 2 * maxAccel) * 0.999999;
    return moveDistanceProfile(distanceTarg, angleTarget, newVmax, maxAccel);
  }

  double currAngle = robot::odometry.getPose().heading.convert(okapi::degree);
  double targetAngle = angleTarget.convert(okapi::degree);
  if (strafing) {
    targetAngle += 90;
  }
  targetAngle = 180.0 / 1_pi *
                std::atan2(std::sin(targetAngle * 1_pi / 180.0),
                           std::cos(targetAngle * 1_pi / 180.0));

  double error = targetAngle - currAngle;
  double filteredError = 180.0 / 1_pi *
                         std::atan2(std::sin(error * 1_pi / 180.0),
                                    std::cos(error * 1_pi / 180.0));
  anglePID.setTarget(targetAngle);
  double power = anglePID.calculate(0);

  robot::xDrive.arcade(0.1 * direction, 0);
  pros::delay(120);

  double dCruise = distance - 2 * dAccel;
  double tCruise = dCruise / maxVel;

  for (double t = 0; t < tAccel; t += 0.01) {
    currAngle = robot::odometry.getPose().heading.convert(okapi::degree);
    currAngle = 180.0 / 1_pi *
                std::atan2(std::sin(currAngle * 1_pi / 180.0),
                           std::cos(currAngle * 1_pi / 180.0));
    targetAngle = angleTarget.convert(okapi::degree);
    targetAngle = 180.0 / 1_pi *
                  std::atan2(std::sin(targetAngle * 1_pi / 180.0),
                             std::cos(targetAngle * 1_pi / 180.0));
    robot::odometry.update();

    error = targetAngle - currAngle;
    filteredError = 180.0 / 1_pi *
                    std::atan2(std::sin(error * 1_pi / 180.0),
                               std::cos(error * 1_pi / 180.0));
    anglePID.setTarget(targetAngle);
    power = anglePID.calculate(0);
    // printf("Power: %1.2f, error: %1.2f\n", power, filteredError);

    double v = maxAccel * t;
    moveDriveLinearVelocity((v * direction)/* + power*/, (v * direction)/* - power*/,
                            strafing);
    pros::delay(10);
  }

  for (double t = 0; t < tCruise; t += 0.01) {
    currAngle = robot::odometry.getPose().heading.convert(okapi::degree);
    currAngle = 180.0 / 1_pi *
                std::atan2(std::sin(currAngle * 1_pi / 180.0),
                           std::cos(currAngle * 1_pi / 180.0));
    targetAngle = angleTarget.convert(okapi::degree);
    targetAngle = 180.0 / 1_pi *
                  std::atan2(std::sin(targetAngle * 1_pi / 180.0),
                             std::cos(targetAngle * 1_pi / 180.0));
    robot::odometry.update();

    error = targetAngle - currAngle;
    filteredError = 180.0 / 1_pi *
                    std::atan2(std::sin(error * 1_pi / 180.0),
                               std::cos(error * 1_pi / 180.0));
    anglePID.setTarget(targetAngle);
    power = anglePID.calculate(0);

    moveDriveLinearVelocity((maxVel * direction)/* + power*/,
                            (maxVel * direction)/* - power*/, strafing);
    pros::delay(10);
  }

  for (double t = 0; t < tAccel; t += 0.01) {
    currAngle = robot::odometry.getPose().heading.convert(okapi::degree);
    currAngle = 180.0 / 1_pi *
                std::atan2(std::sin(currAngle * 1_pi / 180.0),
                           std::cos(currAngle * 1_pi / 180.0));
    targetAngle = angleTarget.convert(okapi::degree);
    targetAngle = 180.0 / 1_pi *
                  std::atan2(std::sin(targetAngle * 1_pi / 180.0),
                             std::cos(targetAngle * 1_pi / 180.0));
    robot::odometry.update();

    error = targetAngle - currAngle;
    filteredError = std::atan2(std::sin(error * 1_pi / 180.0),
                               std::cos(error * 1_pi / 180.0));
    anglePID.setTarget(targetAngle);
    power = anglePID.calculate(0);

    double v = maxVel - maxAccel * t;
    moveDriveLinearVelocity((v * direction)/* + power*/, (v * direction)/* - power*/,
                            strafing);
    pros::delay(10);
  }

  robot::xDrive.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
  moveDriveLinearVelocity(0, 0);
  pros::delay(150);
  profileDone = true;
}

}  // namespace drive
}  // namespace subsystem