#pragma once
#include <optional>
#include "structs.hpp"
#include "xdrive.hpp"

namespace motion {
using namespace structs;

struct XDriveTarget {
  Pose targetPose;
  double angularVelocity;
  PIDGains straightGains;
  PIDGains turnGains;

  XDriveTarget(Pose targetPose, double angularVelocity, PIDGains straightGains,
               PIDGains turnGains)
      : targetPose(targetPose),
        angularVelocity(angularVelocity),
        straightGains(straightGains),
        turnGains(turnGains) {}
};

class XDriveController {
 public:
  /**
   * turnGain
   */
  XDriveController(XDrive &drive, PIDGains &straightGains, PIDGains &turnGains,
                   double angularVelocity);

  void addTarget(Pose target, std::optional<double> angularVelocity,
                 std::optional<PIDGains> straight,
                 std::optional<PIDGains> turn);

 private:
  XDrive drive;
  PIDGains straightGains;
  PIDGains turnGains;
  double omega;

  std::vector<XDriveTarget> targets;

  static void trampoline(void *instance);
  void run();
};
}  // namespace motion