#include "intake.hpp"
#include "motion/pid.hpp"
#include "subsystem/rd4b.hpp"
#include "subsystem/tray.hpp"

namespace subsystem {
namespace intake {

state currState = state::manual;
okapi::ControllerButton intake(okapi::ControllerDigital::R1);
okapi::ControllerButton outtake(okapi::ControllerDigital::R2);

void init() {
  pros::Task(run, nullptr, "intake");
  // = state::manual;
}

double slewTarget = 0.0;
double currPower = 0.0;
double slewRate = 2000;  // mV per 10ms

void moveSlew(double voltage) { slewTarget = voltage; }

void run(void* p) {
  while (true) {
    // if (fabs(slewTarget) <  fabs(currPower)) {
    //   if (fabs(currPower) - fabs(slewRate) < fabs(slewTarget)) {
    //     currPower = slewTarget;
    //   } else {
    //     currPower -= slewRate;
    //   }
    // }

    // if (fabs(slewTarget) >  fabs(currPower)) {
    //   if (fabs(currPower) + fabs(slewRate) > fabs(slewTarget)) {
    //     currPower = slewTarget;
    //   } else {
    //     currPower += slewRate;
    //   }
    // }

    double slewError = slewTarget - currPower;
    motion::PID slewPID(0.2, 0.0, 0.0);
    slewPID.setTarget(slewTarget);
    currPower += slewPID.calculate(currPower);
    if (fabs(slewError) < 100) {
      currPower = slewTarget;
    }

    currPower = std::clamp(currPower, -12000.0, 12000.0);

    if (currPower == 0) {
      robot::intakeGroup.moveVelocity(0);
    } else {
      robot::intakeGroup.moveVoltage(currPower);
    }

    switch (currState) {
      case state::manual:
      {
        double speed = 12000.0;
        if (robot::lift.getPosition() > rd4b::targets::upThreshold) {
          speed = 6000.0;
        }
        if (intake.isPressed() || outtake.isPressed()) {
          robot::intakeGroup.moveVoltage(
              (intake.isPressed() - outtake.isPressed()) * speed);
        } else {
          robot::intakeGroup.setBrakeMode(
              okapi::AbstractMotor::brakeMode::brake);
          moveSlew(0);
        }
      } break;
      case state::free:
        robot::intakeGroup.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
        moveSlew(0);
        break;
      case state::in:
        moveSlew(12000);
        break;
      case state::out:
        moveSlew(-12000);
        break;
      case state::placing:
        robot::intakeGroup.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
        if (intake.isPressed() || outtake.isPressed()) {
          moveSlew((intake.isPressed() - outtake.isPressed()) * 11000.0);
        } else {
          robot::intakeGroup.setBrakeMode(
              okapi::AbstractMotor::brakeMode::brake);
          if (robot::tilt.getActualVelocity() < 0) {
            moveSlew(1500);
          } else {
            if (robot::tilt.getPosition() < tray::intakeIntersect) {
              moveSlew(0);
            } else {
              moveSlew(-2500);
            }
          }
          // robot::intakeGroup.moveVelocity(
          //     10 * ((robot::tilt.getActualVelocity() < 0) ? 1 : -1));
        }
        break;
    }

    pros::delay(10);
  }
}

void in() { currState = state::in; }

void out() { currState = state::out; }

void free() { currState = state::free; }

void manual() { currState = state::manual; }

void changeState(state state) { currState = state; }

}  // namespace intake
}  // namespace subsystem