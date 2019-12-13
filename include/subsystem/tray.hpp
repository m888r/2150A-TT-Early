#pragma once
#include "okapi/api.hpp"
#include "structs.hpp"
#include "motion/profiling.hpp"
#include "motion/sCurveProfile.hpp"
#include "motion/pid.hpp"
#include "config.hpp"
#include "subsystem/intake.hpp"

//#include "moreunits.hpp"

namespace subsystem {
namespace tray {

//using namespace okapi;
enum targets {
  placed = 1335, // 1320 fully vertical
  intakeIntersect = 700,
  normal = 0,
  hightower = 30
};

int getTarget(targets targ);

enum class mode { placing, standby, hightower, holding, resetting};
extern mode currMode;
extern mode lastMode;

enum class state { running, disabled, holding}; // not using anymore
extern state currState;
extern state lastState;

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