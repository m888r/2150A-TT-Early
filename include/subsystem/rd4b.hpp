#pragma once
#include "config.hpp"
#include "okapi/api.hpp"

namespace subsystem {
namespace rd4b {
enum class state { resetting, targeting, holding, manual, up, down, placing };
enum targets {
  downTarget = 0,
  upThreshold = (70 * 7),
  smallTowerTarget = (70 * 7),
  mediumTowerTarget = (90 * 7),
  cubeStackTarget = (100 * 7)
};
enum heightModes { bottom = 0, small = 1, medium = 2, cubeStack = 3 };
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

void rd4bModeStateMachine();

void waitUntilSettled();

state getState();

}  // namespace rd4b
}  // namespace subsystem