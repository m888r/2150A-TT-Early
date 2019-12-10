#pragma once
#include <optional>
#include <queue>
#include "structs.hpp"
#include "xdrive.hpp"
#include "odometry.hpp"

namespace motion {
using namespace structs;

struct XDriveTarget {
  Pose targetPose;
  double angularVelocity;
  PIDGains straightGains;
  PIDGains turnGains;
  okapi::QAngle pidThreshold;

  XDriveTarget();

  XDriveTarget(Pose targetPose, double angularVelocity, PIDGains straightGains,
               PIDGains turnGains, okapi::QAngle pidThreshold)
      : targetPose(targetPose),
        angularVelocity(angularVelocity),
        straightGains(straightGains),
        turnGains(turnGains), pidThreshold(pidThreshold) {}
};

class XDriveController {
 public:
  /**
   * turn PID only runs once it's close to the target heading, otherwise constant omega
   */
  XDriveController(Odometry &odometry, XDrive &drive, PIDGains &straightGains, PIDGains &turnGains,
                   double angularVelocity, okapi::QAngle pidThreshold);

  void addTarget(Pose target, std::optional<double> angularVelocity,
                 std::optional<PIDGains> straight,
                 std::optional<PIDGains> turn, std::optional<okapi::QAngle> pidThreshold);

  void runNextTarget();

  bool isSettled(double distanceThreshold, double angleThreshold);
  void waitUntilSettled(double distanceThreshold, double angleThreshold);

  void disable();
  void enable();

 private:
  Odometry odometry;
  XDrive drive;
  PIDGains straightGains;
  PIDGains turnGains;
  double omega;
  okapi::QAngle defaultPIDThreshold;

  std::queue<XDriveTarget> targets;
  XDriveTarget currTarget;

  pros::Task XDriveControlTask;

  std::atomic<bool> enabled = false;
  std::atomic<bool> isDone = true;

  static void trampoline(void *instance);
  void run();
};
}  // namespace motion