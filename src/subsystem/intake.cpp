#include "intake.hpp"

namespace subsystem {
namespace intake {

state currState = state::manual;
okapi::ControllerButton intake(okapi::ControllerDigital::R1);
okapi::ControllerButton outtake(okapi::ControllerDigital::R2);

void init() {
  pros::Task(run, nullptr, "intake");
  currState = state::manual;
}

void run(void* p) {
  while (true) {
    switch (currState) {
      case state::manual:
        if (intake.isPressed() || outtake.isPressed()) {
          robot::intakeGroup.moveVoltage(
              (intake.isPressed() - outtake.isPressed()) * 12000.0);
        } else {
          robot::intakeGroup.setBrakeMode(
              okapi::AbstractMotor::brakeMode::brake);
          robot::intakeGroup.moveVelocity(0);
        }
        break;
      case state::free:
        robot::intakeGroup.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
        robot::intakeGroup.moveVoltage(0);
        break;
      case state::in:
        robot::intakeGroup.moveVelocity(200);
        break;
      case state::out:
        robot::intakeGroup.moveVelocity(-200);
        break;
      case state::placing:
        robot::intakeGroup.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
        if (intake.isPressed() || outtake.isPressed()) {
          robot::intakeGroup.moveVoltage(
              (intake.isPressed() - outtake.isPressed()) * 12000.0);
        } else {
          robot::intakeGroup.setBrakeMode(
              okapi::AbstractMotor::brakeMode::brake);
          robot::intakeGroup.moveVelocity(0);
        }
        // robot::intakeGroup.moveVelocity(10 *
        // ((robot::tilt.getActualVelocity() < 0) ? -1 : 1));
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