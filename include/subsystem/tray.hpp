#pragma once
#include "okapi/api.hpp"
#include "structs.hpp"
#include "motion/profiling.hpp"
#include "motion/sCurveProfile.hpp"
#include "motion/pid.hpp"

//#include "moreunits.hpp"

namespace subsystem {
namespace tray {

//using namespace okapi;
enum targets {
  placed,
  normal,
  hightower
};

int getTarget(targets targ);

enum class mode { placing, targeting };
mode currMode;
mode lastMode;

enum class state { running, disabled, holding};
state currState = state::disabled;
state lastState = state::disabled;

extern okapi::MotorGroup trayMotor;
extern structs::KinematicConstraints trayConstraints;

extern motion::PID trayPID;

// extern units::angular_velocity::revolutions_per_minute_t velConstraint;
// extern units::angular_acceleration::revolutions_per_minute2_t accelConstraint;
// extern units::angular_jerk::revolutions_per_minute3_t jerkConstraint;

extern double velConstraint;
extern double accelConstraint;
extern double jerkConstraint;

void init();

void run(void* p);
void moveMotionProfile(double target);
void movePID(double target);

void changeMode(mode mode);

double ikVelocity(double omega_in);

}  // namespace tray
}  // namespace subsystem