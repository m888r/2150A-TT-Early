#include "auton/autos.hpp"
#include "auton/asyncActions.hpp"
#include "config.hpp"
#include "subsystem/drive.hpp"
#include "subsystem/intake.hpp"
#include "subsystem/rd4b.hpp"
#include "subsystem/tray.hpp"

using namespace okapi::literals;
using namespace subsystem;

// blueclose
void blueClosePaths() {}

// auton for blue side closer to double zone
void blueCloseAuto() {
  robot::lift.tarePosition();
  robot::odometry.setPose({0_in, 0_in, 0_deg});
  intake::in();
  drive::moveDistanceProfile(0.1);
  Async({
    pros::delay(100);
    rd4b::moveTarget(300);
    pros::delay(400);
    rd4b::moveTarget(0);
  })
  pros::delay(1300);
  rd4b::moveTarget(1280);
  pros::delay(500);
  drive::moveDistanceProfile(0.65);
  rd4b::moveTarget(0);
  pros::delay(50);
  okapi::Timer timer;
  timer.placeMark();
  while (rd4b::getState() != rd4b::state::holding &&
         timer.getDtFromMark() < 2000_ms) {
    pros::delay(10);
  }
  pros::delay(350);
  okapi::QAngle placeAngle = 125_deg;
  drive::turnTo(placeAngle);
  robot::odometry.setPose({0_in, 0_in, 0_deg});
  pros::delay(10);
  drive::moveDistanceProfile(1.02);
  tray::changeMode(tray::mode::placing);
  pros::delay(2000);
  drive::moveDistanceProfile(-0.5, 0_deg, 0.5);
  tray::changeMode(tray::mode::standby);
  pros::delay(1000);

}

// bluefar
void blueFarPaths() {}

void blueFarAuto() {
  robot::lift.tarePosition();
  robot::odometry.setPose({0_in, 0_in, 0_deg});
  intake::in();
  drive::moveDistanceProfile(0.1);
  Async({
    pros::delay(100);
    rd4b::moveTarget(300);
    pros::delay(400);
    rd4b::moveTarget(0);
  })
  pros::delay(1300);
  intake::free();
  drive::moveDistanceProfile(0.65, 0_deg, 0.7);
  intake::out();
  pros::delay(100);
}

// redclose
void redClosePaths() {}

void redCloseAuto() {
  robot::lift.tarePosition();
  robot::odometry.setPose({0_in, 0_in, 0_deg});
  intake::in();
  drive::moveDistanceProfile(0.1);
  Async({
    pros::delay(100);
    rd4b::moveTarget(300);
    pros::delay(400);
    rd4b::moveTarget(0);
  })
  pros::delay(1300);
  rd4b::moveTarget(1280);
  pros::delay(500);
  drive::moveDistanceProfile(0.65);
  rd4b::moveTarget(0);
  pros::delay(50);
  okapi::Timer timer;
  timer.placeMark();
  while (rd4b::getState() != rd4b::state::holding &&
         timer.getDtFromMark() < 2000_ms) {
    pros::delay(10);
  }
  pros::delay(350);
  okapi::QAngle placeAngle = -125_deg;
  drive::turnTo(placeAngle);
  robot::odometry.setPose({0_in, 0_in, 0_deg});
  pros::delay(10);
  drive::moveDistanceProfile(1.02);
  tray::changeMode(tray::mode::placing);
  pros::delay(2000);
  drive::moveDistanceProfile(-0.5, 0_deg, 0.5);
  tray::changeMode(tray::mode::standby);
  pros::delay(1000);
}

// redfar
void redFarPaths() {}

void redFarAuto() {
  robot::lift.tarePosition();
  robot::odometry.setPose({0_in, 0_in, 0_deg});
  intake::in();
  drive::moveDistanceProfile(0.1);
  Async({
    pros::delay(100);
    rd4b::moveTarget(300);
    pros::delay(400);
    rd4b::moveTarget(0);
  })
  pros::delay(1300);
  drive::moveDistanceProfile(0.65, 0_deg, 0.7);
  intake::out();
  pros::delay(100);
}

void redFarAutoParkOnly() {}

// REMEMBER TO REVERSE
void skills() {}