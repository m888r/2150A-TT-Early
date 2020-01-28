#include "tray.hpp"

namespace subsystem {
namespace tray {

mode currMode = mode::holding;
mode lastMode = currMode;
mode nextMode = currMode;
state currState = state::disabled;
state lastState = state::disabled;

// using namespace units::literals;

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

motion::PID trayPID(0.0012, 0.0, 0.0, 0);  // was 0.0022 was 0.00195
motion::PID firstMovePID(0.0025, 0.0, 0.0, 0); // was 0.00195

void init() {
  pros::Task trayTask(run, nullptr, "Tray");
  currState = state::running;
  currMode = mode::holding;
  robot::tilt.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
  robot::tilt.tarePosition();
  robot::tilt.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
}

void run(void* p) {
  motion::SCurveProfile profile(trayConstraints, 0);
  okapi::Timer timer;
  okapi::Timer standbyTimer;
  timer.placeMark();

  bool motionProfileComplete = false;
  int motionprofileDirection = 1;
  bool startedPlacing = false;
  bool motionProfileDisabled = true;
  while (true) {
    if (currMode != mode::placing) {
      motionProfileComplete = false;
      motionprofileDirection = 1;
      startedPlacing = false;
    }

    switch (currMode) {
      case mode::placing: {
        // printf("%d: Entered placing\n", pros::millis());
        int error = (placed)-robot::tilt.getPosition();
        if (/*lastMode != mode::placing*/ !startedPlacing) {
          printf("%d: Entered start of place, ", pros::millis());
          startedPlacing = true;
          motionProfileComplete = false;
          profile = motion::SCurveProfile(trayConstraints, abs(error / 360.0));
          printf(
              "Generated Profile, current pos: %1.2f, target: %1.2f, error: "
              "%1.2f",
              robot::tilt.getPosition(), (placed), error);
          timer.placeMark();
          if (error < 0) {  // reverse the profile if the thing is backwards
            motionprofileDirection = -1;
          } else {
            motionprofileDirection = 1;
          }
        }
        subsystem::intake::changeState(subsystem::intake::state::placing);
        if ((!motionProfileComplete || abs(error) > 150) &&
            !motionProfileDisabled) {
          printf("%d: Error: %d, ", error, pros::millis());
          double totalDt = timer.getDtFromMark().convert(okapi::minute);
          if (totalDt > profile.getTotalTime()) {
            totalDt = profile.getTotalTime();
            motionProfileComplete = true;
          }
          auto output = profile.calculate(totalDt);
          robot::tilt.moveVelocity(ikVelocity(output.velocity) *
                                   motionprofileDirection);
          printf("moving at v=%1.2f\n", output.velocity * 100000000);
          if (motionProfileComplete) {
            printf("%d: profile complete!\n", pros::millis());
          }
        } else if (motionProfileDisabled) {
          trayPID.setTarget((placed));
          double voltage = trayPID.calculate(robot::tilt.getPosition());
          voltage = std::min(voltage, 1.0);
          voltage = std::max(voltage, -1.0);
          robot::tilt.moveVelocity(voltage * 200.0);
          // printf("%d: Voltage: %1.2f, ", pros::millis(), voltage * 12000.0);
          // printf("Error: %d, ", error);
          // printf("Pos: %1.2f\n", robot::tilt.getPosition());
          //robot::tilt.moveAbsolute(placed, 100);
        }

        if (abs(error) < 50) {
          printf("aight imma head out\n");
          nextMode = mode::holding;
        }
      } break;
      case mode::standby:
        // subsystem::intake::free();
        subsystem::intake::changeState(subsystem::intake::state::placing);
        robot::tilt.moveAbsolute((normal), 200);
        if (abs(robot::tilt.getTargetPosition() - robot::tilt.getPosition()) <
            50) {
          nextMode = mode::holding;
        }
        break;
      case mode::resetting:
        robot::tilt.moveVelocity(-50);
        if (lastMode != mode::resetting) {
          standbyTimer.placeMark();
        }
        if (abs(robot::tilt.getActualVelocity()) < 1 &&
            standbyTimer.getDtFromMark().convert(okapi::millisecond) > 100) {
          robot::tilt.tarePosition();
          robot::tilt.moveVoltage(0);
          nextMode = mode::holding;
        }
        break;
      case mode::prepared: {
        firstMovePID.setTarget(intakeIntersect);
        double power = firstMovePID.calculate(robot::tilt.getPosition());
        power = std::clamp(power, -1.0, 1.0);
        robot::tilt.moveVoltage(power * 12000.0);
      } break;
      case mode::spinnable: {
        firstMovePID.setTarget(balancedCoM);
        double power = firstMovePID.calculate(robot::tilt.getPosition());
        power = std::clamp(power, -1.0, 1.0);
        robot::tilt.moveVoltage(power * 12000.0);
      } break;
      case mode::holding:
        if (!pros::competition::is_autonomous()) {
          subsystem::intake::manual();
        }

        robot::tilt.moveVelocity(0);
        break;
    }

    lastMode = currMode;
    currMode = nextMode;
    pros::delay(10);
  }
}  // namespace tray

void moveMotionProfile(double target) {}

void movePID(double target) {}

// int getTarget(targets targ) {
//   switch (targ) {
//     case placed:
//       return 1810;
//       break;
//     case intakeIntersect:
//       return 950;
//       break;
//     case normal:
//       return 0;
//       break;
//     case hightower:
//       return 30;
//   }
// }

void changeMode(mode mode) {
  lastMode = currMode;
  nextMode = mode;
}

double ikVelocity(double omega_in) { return omega_in; }

}  // namespace tray
}  // namespace subsystem