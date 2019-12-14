#include "subsystem/lift.hpp"

namespace subsystem {

Lift::Lift(std::shared_ptr<okapi::MotorGroup> liftMtrs, okapi::ControllerDigital up, okapi::ControllerDigital down)
    : running(true), liftTask(trampoline, this, "Lift"), upBtn(up), downBtn(down) {
  liftMtrs_ = liftMtrs;
}

void Lift::trampoline(void* instance) {
  static_cast<Lift*>(instance)->run();
}

void Lift::run() {
  int currTarget = 0;
  while (true) {
    switch (currState) {
      case State::coasting:
        liftMtrs_->moveVoltage(0);
        break;
      case State::directcontrol:
        //liftMtrs_->moveVoltage((upBtn.isPressed() * 12000) - (upBtn.isPressed() * 12000));
        liftMtrs_->moveVelocity((upBtn.isPressed() * 200) - (upBtn.isPressed() * 200));

        break;
      case State::targeting:
        // move to targets[currTarget]
        // implement a way to change target
        // but also a way to do with controller
        if (upBtn.changedToPressed()) {
          currTarget++;
        } else if (downBtn.changedToPressed()) {
          currTarget--;
        }
        if (currTarget >= targets.size()) {
          currTarget = 0;
        } else if (currTarget < 0) {
          currTarget = targets.size() - 1;
        }
    }

    pros::delay(10);
  }
}

}  // namespace subsystem