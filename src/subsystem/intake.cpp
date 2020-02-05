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

    // if (currPower == 0) {
    //   robot::intakeGroup.moveVelocity(0);
    // } else {
    //   robot::intakeGroup.moveVoltage(currPower);
    // }

    switch (currState) {
      case state::disabled:
        break;
      case state::manual:
      {
        double speed = 12000.0;
        //double speed = 200;
        if (robot::lift.getPosition() > rd4b::targets::upThreshold) {
          speed = 6000.0;
          //speed = 100;
          //printf("CHANGED SPEED TO: %1.2f", speed);
        }
        // if (intake.isPressed() || outtake.isPressed()) {
        //   robot::intakeGroup.moveVoltage(
        //       (intake.isPressed() - outtake.isPressed()) * speed);
        // }
        if (intake.isPressed()) {
          robot::intakeGroup.moveVoltage(12000);
        } else if (outtake.isPressed()) {
          robot::intakeGroup.moveVoltage(-speed); 
        } else {
          robot::intakeGroup.setBrakeMode(
              okapi::AbstractMotor::brakeMode::brake);
          robot::intakeGroup.moveVelocity(0);
        }
      } break;
      case state::free:
        robot::intakeGroup.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
        robot::intakeGroup.moveVelocity(0);
        break;
      case state::in:
        robot::intakeGroup.moveVoltage(12000);
        break;
      case state::out:
        robot::intakeGroup.moveVoltage(-12000);
        break;
      case state::outPosition:
        robot::intakeGroup.moveRelative(-220, 100);
        break;
      case state::holding:
        robot::intakeGroup.moveVoltage(1000);
        break;
      case state::placing:
        //printf("IS IN PLACING MODE");
        robot::intakeGroup.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
        if (intake.isPressed() || outtake.isPressed()) {
          robot::intakeGroup.moveVelocity((intake.isPressed() - outtake.isPressed()) * 100.0);
        }
        //robot::intakeGroup.moveVoltage(800);
        // if (intake.isPressed() || outtake.isPressed()) {
        //     robot::intakeGroup.moveVoltage((intake.isPressed() - outtake.isPressed()) * 12000.0);
        // } else {
        //   robot::intakeGroup.setBrakeMode(
        //       okapi::AbstractMotor::brakeMode::brake);
        //   if (robot::tilt.getActualVelocity() < 0) {
        //     robot::intakeGroup.moveVoltage(1500);
        //   } else {
        //     if (robot::tilt.getPosition() < tray::intakeIntersect) {
        //       robot::intakeGroup.moveVoltage(0);
        //     } else {
        //       robot::intakeGroup.moveVoltage(-2500);
        //     }
        //   }
        //   // robot::intakeGroup.moveVelocity(
        //   //     10 * ((robot::tilt.getActualVelocity() < 0) ? 1 : -1));
        // }
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