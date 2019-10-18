#include "subsystem/drive.hpp"

namespace subsystem {
namespace drive {

void turnTo(okapi::QAngle angle) {
  using namespace okapi::literals;

  auto su = okapi::SettledUtilFactory::create(0.7, 0.4, 200_ms);
  robot::xDrive.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);

  auto angleRad = -angle.convert(okapi::radian);

  motion::PID pid(0.027, 0.0003, 1.6);
  pid.setMaxErrorToIntegrate(15);
  pid.setIntegralReset(true);
  pid.setTarget(angle.convert(okapi::degree));

  while (!su.isSettled(pid.getError())) {
    robot::odometry.update();
    double out =
        pid.calculate(robot::odometry.getPose().theta.convert(okapi::degree));
    robot::xDrive.driveVectorVoltage(0, out);
    pros::delay(10);
  }

  robot::xDrive.stop();
}

void moveDriveLinearVelocity(double vel) {
  using namespace okapi::literals;

  double rpm = vel / (1_pi * robot::wheelDiameter.convert(okapi::meter)) * (60.0 /*sec*/ / 1.0 /*min*/);
  robot::xDrive.driveVector(rpm, 0);
}

void moveDistanceProfile(double distance, bool backwards = false, double maxVel = 1.05, double maxAccel = 2.0) {
    double tAccel = maxVel / maxAccel;
    double dAccel = 1.0 / 2.0 * tAccel * maxVel;

    if (dAccel * 2.0 > distance) {
        double dAccelDesired = distance / 2.0;
        maxVel = sqrt(dAccelDesired * 2 * maxAccel);
        // return moveForward(distance, newVmax, maxAccel);
    }

    int direction = (backwards) ? -1 : 1;

    double dCruise = distance - 2 * dAccel;
    double tCruise = dCruise / maxVel;

    for (double t = 0; t < tAccel; t += 0.001) {
        double v = maxAccel * t;
        moveDriveLinearVelocity(v * direction);
        pros::delay(10);
    }

    for (double t = 0; t < tCruise; t += 0.001) {
        moveDriveLinearVelocity(maxVel * direction);
        pros::delay(10);
    }

    for (double t = 0; t < tAccel; t += 0.001) {
        double v = maxVel - maxAccel * t;
        moveDriveLinearVelocity(v * direction);
        pros::delay(10);
    }

    robot::xDrive.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
    moveDriveLinearVelocity(0);
}

}  // namespace turning
}  // namespace subsystem