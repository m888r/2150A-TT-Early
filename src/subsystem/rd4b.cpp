#include "rd4b.hpp"

namespace subsystem {
namespace rd4b {
okapi::ControllerButton up(okapi::ControllerDigital::L1);
okapi::ControllerButton down(okapi::ControllerDigital::L2);

int currTarget = 0;
state currState = state::holding;
state lastState = currState;

pros::Mutex stateMutex;

void init() {
  robot::lift.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
  robot::lift.setEncoderUnits(okapi::AbstractMotor::encoderUnits::degrees);
  robot::lift.tarePosition();
  currState = state::holding;
  pros::Task rd4bTask(run, nullptr, "RD4B");
}

void run(void* p) {
  okapi::Timer timer;
  bool startedReset = false;

  while (true) {
    robot::lift.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);

    if (currState != state::resetting) {
      startedReset = false;
    }

    switch (currState) {
      case state::holding:
        robot::lift.moveVelocity(0);
        break;
      case state::targeting:
        robot::lift.moveAbsolute(currTarget, 200);
        if (abs(robot::lift.getTargetPosition() - robot::lift.getPosition()) <
            25) {
          stateMutex.take(TIMEOUT_MAX);
          currState = state::holding;
          stateMutex.give();
        }
        break;
      case state::resetting:
        if (!startedReset) {
          timer.placeMark();
          startedReset = true;
        }
        robot::lift.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
        robot::lift.moveVoltage(-3000);
        if (robot::lift.isStopped() &&
            timer.getDtFromMark().convert(okapi::millisecond) > 200) {
          robot::lift.moveVelocity(0);
          robot::lift.tarePosition();
          stateMutex.take(TIMEOUT_MAX);
          currState = state::holding;
          stateMutex.give();
        }
        break;
      case state::manual:
        if (up.isPressed() || down.isPressed()) {
          robot::lift.moveVoltage(12000.0 * (up.isPressed() - down.isPressed()));
        } else {
          robot::lift.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
          robot::lift.moveVelocity(0);
        }
        break;
    }
    lastState = currState;
    pros::delay(10);
  }
}

void moveTarget(int target) {
  stateMutex.take(TIMEOUT_MAX);
  currState = state::targeting;
  stateMutex.give();
  currTarget = target;
}

void changeState(state state) {
  stateMutex.take(TIMEOUT_MAX);
  currState = state;
  stateMutex.give();
}

void waitUntilSettled() {
  while (abs(robot::lift.getTargetPosition() - robot::lift.getPosition()) > 50) {
    pros::delay(10);
  }
}

state getState() { return currState;}
}  // namespace rd4b
}  // namespace subsystem