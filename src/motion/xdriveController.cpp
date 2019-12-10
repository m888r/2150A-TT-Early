#include "xdriveController.hpp"
#include "pid.hpp"

namespace motion {
XDriveController::XDriveController(Odometry &odometry, XDrive &drive,
                                   PIDGains &straightGains, PIDGains &turnGains,
                                   double angularVelocity,
                                   okapi::QAngle pidThreshold)
    : drive(drive),
      odometry(odometry),
      straightGains(straightGains),
      turnGains(turnGains),
      omega(angularVelocity),
      defaultPIDThreshold(pidThreshold),
      XDriveControlTask(trampoline, this, "X Drive Control") {}

void XDriveController::addTarget(Pose target,
                                 std::optional<double> angularVelocity,
                                 std::optional<PIDGains> straight,
                                 std::optional<PIDGains> turn,
                                 std::optional<okapi::QAngle> pidThreshold) {
  targets.push(XDriveTarget(
      target, angularVelocity.value_or(omega), straight.value_or(straightGains),
      turn.value_or(turnGains), pidThreshold.value_or(defaultPIDThreshold)));
}

void XDriveController::runNextTarget() {
  if (!targets.empty()) {
    // currTarget = targets.pop();
    enabled.store(true);
    isDone.store(false);
  }
}

bool XDriveController::isSettled(double distanceThreshold,
                                 double angleThreshold) {
  return (odometry.getPose().heading.convert(okapi::degree));
}

void XDriveController::waitUntilSettled(double distanceThreshold,
                                        double angleThreshold) {}

void XDriveController::enable() { enabled = true; }

void XDriveController::disable() { enabled = false; }

void XDriveController::run() {
  while (true) {
    if (!(enabled.load())) {
      // do nothing

      //} else if (targets.empty()) {
      // stop drive
    } else if (isDone.load()) {
    } else {
      auto target = currTarget;
      PID straightPID(target.straightGains.kP, target.straightGains.kI,
                      target.straightGains.kD);
      PID turnPID(target.turnGains.kP, target.turnGains.kI,
                  target.turnGains.kD);
      auto targetPose = target.targetPose;
      auto targetPosition = targetPose.position;
      double w = target.angularVelocity;
      double linearDistance =
          odometry.getDistanceTo(targetPosition.getX(), targetPosition.getY())
              .convert(okapi::meter);

      

      straightPID.setTarget(0);
      double linearVelocity = straightPID.calculate(linearDistance);

      // drive fk

      
    }

    pros::delay(10);
  }
}

void XDriveController::trampoline(void *instance) {
  static_cast<XDriveController *>(instance)->run();
}

}  // namespace motion