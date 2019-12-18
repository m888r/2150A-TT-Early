#pragma once
#include "okapi/api.hpp"
#include "config.hpp"

namespace subsystem {
namespace rd4b {
enum class state { resetting, targeting, holding, manual };
enum targets {downTarget = 0, upThreshold = (52 * 7)};
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
void moveTarget(int target, int desiredSpeed = 200);
void changeState(state state);

void waitUntilSettled();

state getState();

}  // namespace rd4b
}  // namespace subsystem