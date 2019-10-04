#include "tray.hpp"

namespace subsystem {
namespace tray {

// using namespace units::literals;

okapi::MotorGroup trayMotor = okapi::MotorGroup({9});

// units::angular_velocity::revolutions_per_minute_t velConstraint =
// 200_rad_per_s; units::angular_acceleration::revolutions_per_minute2_t
// accelConstraint = 200_rpm / 1.5_s;
// units::angular_jerk::revolutions_per_minute3_t jerkConstraint = 200_rpm
// / 1.5_s / 0.25_s;

double velConstraint = 200;  // 200 rpm
double accelConstraint =
    200.0 * ((1.0 / 1.5 /*sec*/) *
             (60.0 /*sec*/ / 1.0 /*min*/));  // 200rpm / 1.5s -> rpm^2
double jerkConstraint =
    200 * ((1.0 / 1.5 /*sec*/) * (60.0 /*sec*/ / 1.0 /*min*/)) *
    ((1.0 / 0.25 /*sec*/) *
     (60.0 /*sec*/ / 1.0 /*min*/));  // 200rpm / 1.5s / 0.25s -> rpm^3

structs::KinematicConstraints trayConstraints(velConstraint, accelConstraint,
                                              jerkConstraint);

motion::PID trayPID(0.009, 0.0, 0.0, 0.0);

void init() {
  pros::Task trayTask(run, nullptr, "Tray");
  currState = state::running;
  currMode = mode::targeting;
}

void run(void* p) {
  motion::SCurveProfile profile(trayConstraints, 0);
  okapi::Timer timer;
  timer.placeMark();

  bool motionProfileComplete = false;
  mode nextMode = currMode;
  int motionprofileDirection = 1;
  while (true) {
    switch (currState) {
      case state::running:
        switch (currMode) {
          case mode::placing: {
            int error = getTarget(placed) - trayMotor.getPosition();
            if (lastMode != mode::placing) {
              motionProfileComplete = false;
              profile = motion::SCurveProfile(trayConstraints, abs(error));
              timer.placeMark();
              if (error < 0) { // reverse the profile if the thing is backwards
                motionprofileDirection = -1;
              } else {
                motionprofileDirection = 1;
              }
            }
            if (!motionProfileComplete || abs(error) > 150) {
              double totalDt =
                  timer.getDtFromMark().convert(okapi::minute);
              if (totalDt > profile.getTotalTime()) {
                totalDt = profile.getTotalTime();
                motionProfileComplete = true;
              }
              auto output = profile.calculate(totalDt);
              trayMotor.moveVelocity(ikVelocity(output.velocity) * motionprofileDirection);
            } else {
              double voltage = trayPID.calculate(error);
              trayMotor.moveVoltage(voltage * 12000.0);
            }
          } break;
          case mode::targeting:
            trayMotor.moveAbsolute(getTarget(normal), 200);
            break;
        }
        break;
      case state::holding:
        trayMotor.moveVelocity(0);
        break;
    }

    lastMode = currMode;
    currMode = nextMode;
    lastState = currState;
    pros::delay(10);
  }
}

void moveMotionProfile(double target) {}

void movePID(double target) {}

int getTarget(targets targ) {
  switch (targ) {
    case placed:
      return 170;
      break;
    case normal:
      return 0;
      break;
    case hightower:
      return 30;
  }
}

void changeMode(mode mode) {
  lastMode = currMode;
  currMode = mode;
}

double ikVelocity(double omega_in) {
  return omega_in;
}

}  // namespace tray
}  // namespace subsystem