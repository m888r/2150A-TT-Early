#include "xdriveController.hpp"

namespace motion {
  XDriveController::XDriveController(XDrive &drive, PIDGains &straightGains, PIDGains &turnGains,
                   double angularVelocity)
      : drive(drive),
        straightGains(straightGains),
        turnGains(turnGains),
        omega(angularVelocity) {}

  void XDriveController::addTarget(Pose target, std::optional<double> angularVelocity, std::optional<PIDGains> straight, std::optional<PIDGains> turn) {
    targets.push_back(XDriveTarget(target, angularVelocity.value_or(omega), straight.value_or(straightGains), turn.value_or(turnGains)));
  }
}