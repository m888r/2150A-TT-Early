#include "rd4b.hpp"

namespace subsystem {
namespace rd4b {
okapi::ControllerButton up(okapi::ControllerDigital::L1);
okapi::ControllerButton down(okapi::ControllerDigital::L2);

int currTarget = 0;
state currState = state::holding;
state lastState = currState;
int speed = 200;
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
        robot::lift.moveAbsolute(currTarget, speed);
        if (abs(robot::lift.getTargetPosition() - robot::lift.getPosition()) <
            25) {
          stateMutex.take(TIMEOUT_MAX);
          currState = state::holding;
          stateMutex.give();
        }
        break;
      case state::resetting:
        if (!startedReset && abs(robot::lift.getActualVelocity()) < 1) {
          timer.placeMark();
          startedReset = true;
        } else if (!(abs(robot::lift.getActualVelocity()) < 1)) {
          timer.placeMark();
          startedReset = false;
        }

        robot::lift.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
        robot::lift.moveVoltage(-12000);
        if (abs(robot::tilt.getActualVelocity()) < 1 &&
            timer.getDtFromMark().convert(okapi::millisecond) > 100) {
          robot::lift.moveVelocity(0);
          robot::lift.tarePosition();
          stateMutex.take(TIMEOUT_MAX);
          currState = state::manual;
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
      case state::up:
        robot::lift.moveVoltage(12000.0);
        break;
      case state::down:
        robot::lift.moveVoltage(-12000.0);
        break;
      case state::placing:
        if (lastState != state::placing) { 
          robot::lift.moveRelative(200, 200); // figure out logic to make it relative
        }
        break;
    }
    lastState = currState;
    pros::delay(10);
  }
}

int modeSelector = 0;

void rd4bModeStateMachine() {
  if (up.changedToPressed()) {
    modeSelector++;
  }
  else if (down.changedToPressed()) {
    modeSelector--;
  }

  if (modeSelector > 3) {
    modeSelector = 3;
  }
  if (modeSelector < 0) {
    modeSelector = 0;
  }

  // remember to:
  // change a button to activate state::resetting
  // change state::resetting to not change the state after finishing (and just like hold motor)
  switch (modeSelector) {
    case bottom:
      moveTarget(downTarget);
      break;
    case small:
      moveTarget(smallTowerTarget);
      break;
    case medium:
      moveTarget(mediumTowerTarget);
      break;
    case cubeStack:
      moveTarget(cubeStackTarget);
      break;
  }

}

void moveTarget(int target, int desiredSpeed) {
  stateMutex.take(TIMEOUT_MAX);
  currState = state::targeting;
  stateMutex.give();
  currTarget = target;
  speed = desiredSpeed;
}

void changeState(state state) {
  stateMutex.take(TIMEOUT_MAX);
  currState = state;
  stateMutex.give();
}

void waitUntilSettled() {
  while (std::abs(robot::lift.getTargetPosition() - robot::lift.getPosition()) > 50) {
    pros::delay(10);
  }
}

state getState() { return currState;}
}  // namespace rd4b
}  // namespace subsystem