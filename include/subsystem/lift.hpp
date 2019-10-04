#pragma once

#include "api.h"
#include "okapi/api.hpp"
#include "structs.hpp"

namespace subsystem {
class Lift {
 private:
  static void trampoline(void* instance);
  void run();
  void moveMotionProfile(double target);
  void movePID(double target);

 public:
  Lift(std::shared_ptr<okapi::MotorGroup> liftMtrs, okapi::ControllerDigital up,
       okapi::ControllerDigital down);

  void addPIDGains(double kP, double kI, double kD);
  void addMotionProfileGains(okapi::QSpeed v, okapi::QAcceleration a,
                             okapi::QJerk j);
  void setTargets(std::initializer_list<double> targets);

 protected:
  enum class State { targeting, directcontrol, coasting };

  std::shared_ptr<okapi::MotorGroup> liftMtrs_;
  pros::Task liftTask;
  std::vector<double> targets;
  bool running;
  State currState;
  State lastState;
  okapi::ControllerButton upBtn;
  okapi::ControllerButton downBtn;
};
}  // namespace subsystem