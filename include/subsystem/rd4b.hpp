#pragma once
#include "okapi/api.hpp"
#include "config.hpp"

namespace subsystem {
namespace rd4b {
enum class state { resetting, targeting, holding, manual };
extern int currTarget;
extern state currState;
extern state lastState;

extern pros::Mutex stateMutex;

extern okapi::ControllerButton up;
extern okapi::ControllerButton down;

void init();
void run(void* p);
void moveTargetMotionProfile(double target);
void moveTargetPID(double target);
void moveTarget(int target);
void changeState(state state);

}  // namespace rd4b
}  // namespace subsystem