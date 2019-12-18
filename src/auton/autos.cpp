#include "auton/autos.hpp"
#include "auton/asyncActions.hpp"
#include "config.hpp"
#include "subsystem/drive.hpp"
#include "subsystem/intake.hpp"
#include "subsystem/rd4b.hpp"
#include "subsystem/tray.hpp"
#include "subsystem/xdriveutilities.hpp"

using namespace okapi::literals;
using namespace subsystem;
using namespace robot;
using namespace structs;

void test() {
  odometry.reset();
  pros::delay(50);
  Async({
    drive::moveTo(Pose(2_ft, 0_ft, 180_deg), std::nullopt, std::nullopt,
                  std::nullopt, std::nullopt);
  }) while (!drive::isAtTarget()) {
    printf("Still Waiting\n");
    pros::delay(10);
  }
  drive::stop();
  printf("Stopped\n");
  Async({ drive::moveTo(Pose(0_ft, 0_ft, 0_deg)); }) while (
      !drive::isAtTarget()) {
    printf("Waiting for second movement");
    pros::delay(10);
  }
  printf("Finished second move\n");
  drive::stop();
  // drive::turnTo(90_deg);
  // drive::turnTo(0_deg);
  // drive::turnTo(110_deg);
}

// blueclose
void blueClosePaths() {}

// auton for blue side closer to double zone
void blueCloseAuto() {
  robot::lift.tarePosition();
  odometry.reset();
  pros::delay(50);
  intake::in();
  drive::moveDistanceProfile(0.1);
  Async({
    pros::delay(100);
    rd4b::moveTarget(300);
    pros::delay(400);
    rd4b::moveTarget(0);
  }) pros::delay(1300);
  rd4b::moveTarget(2000, 110); // was 1280
  pros::delay(500);
  // drive::moveDistanceProfile(0.65);
  Async({
    drive::moveTo(Pose(2.4_ft, 0_ft, 0_deg), std::nullopt, std::nullopt,
                  std::nullopt, std::nullopt);
  }) 
  while (!drive::isAtTarget()) { 
    //printf("Still Waiting\n");
    pros::delay(10); 
  }
  drive::stop();
  rd4b::moveTarget(0);
  pros::delay(50);
  okapi::Timer timer;
  timer.placeMark();
  while (rd4b::getState() != rd4b::state::holding &&
         timer.getDtFromMark() < 2000_ms) {
    pros::delay(10);
  }
  tray::changeMode(tray::mode::prepared);
  pros::delay(350);
  okapi::QAngle placeAngle = -110_deg;
  drive::turnTo(placeAngle);

  Async({
    drive::moveTo(Pose(0.38_ft, -1.95_ft, -110_deg), std::nullopt, std::nullopt,
                  std::nullopt, std::nullopt);
  }) 
  pros::delay(300);
  while (!drive::isAtTarget()) { 
    //printf("Still Waiting\n");
    pros::delay(10); 
  }
  drive::stop();

  pros::delay(300);
  tray::changeMode(tray::mode::placing);
  pros::delay(1000);
  rd4b::moveTarget(1200);

  timer.placeMark();
  while (rd4b::getState() != rd4b::state::holding &&
         timer.getDtFromMark() < 2000_ms) {
    pros::delay(10);
  }

  drive::moveDistanceProfile(-0.5, 0_deg, 0.5);
  tray::changeMode(tray::mode::standby);
  pros::delay(1000);
}

// bluefar
void blueFarPaths() {}

void blueFarAuto() {
  robot::lift.tarePosition();
  odometry.reset();
  pros::delay(50);
  intake::in();
  drive::moveDistanceProfile(0.1);
  Async({
    pros::delay(100);
    rd4b::moveTarget(300);
    pros::delay(400);
    rd4b::moveTarget(0);
  }) pros::delay(1300);
  //rd4b::moveTarget(2000, 110); // was 1280
  //pros::delay(500);
  // drive::moveDistanceProfile(0.65);
  Async({
    drive::moveTo(Pose(2.6_ft, 0_ft, 0_deg), std::nullopt, std::nullopt,
                  std::nullopt, std::nullopt);
  }) 
  while (!drive::isAtTarget()) { 
    //printf("Still Waiting\n");
    pros::delay(10); 
  }
  drive::stop();

  Async({
    drive::moveTo(Pose(0.05_ft, 1.0_ft, 50_deg), std::nullopt, std::nullopt,
                  std::nullopt, std::nullopt);
  })
  while (!drive::isAtTarget()) { 
    //printf("Still Waiting\n");
    pros::delay(10);
  }
  drive::stop();

  Async({
    drive::moveTo(Pose(2.6_ft, 1.25_ft, 25_deg), std::nullopt, std::nullopt,
                  std::nullopt, std::nullopt);
  }) 
  while (!drive::isAtTarget()) { 
    //printf("Still Waiting\n");
    pros::delay(10); 
  }
  drive::stop();
}

// redclose
void redClosePaths() {}

void redCloseAuto() {
  robot::lift.tarePosition();
  odometry.reset();
  pros::delay(50);
  intake::in();
  drive::moveDistanceProfile(0.1);
  Async({
    pros::delay(100);
    rd4b::moveTarget(300);
    pros::delay(400);
    rd4b::moveTarget(0);
  }) pros::delay(1300);
  rd4b::moveTarget(2000, 110); // was 1280
  pros::delay(500);
  // drive::moveDistanceProfile(0.65);
  Async({
    drive::moveTo(Pose(2.4_ft, 0_ft, 0_deg), std::nullopt, std::nullopt,
                  std::nullopt, std::nullopt);
  }) 
  while (!drive::isAtTarget()) { 
    //printf("Still Waiting\n");
    pros::delay(10); 
  }
  drive::stop();
  rd4b::moveTarget(0);
  pros::delay(50);
  okapi::Timer timer;
  timer.placeMark();
  while (rd4b::getState() != rd4b::state::holding &&
         timer.getDtFromMark() < 2000_ms) {
    pros::delay(10);
  }
  tray::changeMode(tray::mode::prepared);
  pros::delay(350);
  okapi::QAngle placeAngle = 110_deg;
  drive::turnTo(placeAngle);

  Async({
    drive::moveTo(Pose(0.38_ft, 1.95_ft, 110_deg), std::nullopt, std::nullopt,
                  std::nullopt, std::nullopt);
  }) 
  pros::delay(300);
  while (!drive::isAtTarget()) { 
    //printf("Still Waiting\n");
    pros::delay(10); 
  }
  drive::stop();

  pros::delay(300);
  tray::changeMode(tray::mode::placing);
  pros::delay(1000);
  rd4b::moveTarget(1200);

  timer.placeMark();
  while (rd4b::getState() != rd4b::state::holding &&
         timer.getDtFromMark() < 2000_ms) {
    pros::delay(10);
  }

  drive::moveDistanceProfile(-0.5, 0_deg, 0.5);
  tray::changeMode(tray::mode::standby);
  pros::delay(1000);
}

// redfar
void redFarPaths() {}

void redFarAuto() {
  robot::lift.tarePosition();
  odometry.reset();
  pros::delay(50);
  intake::in();
  drive::moveDistanceProfile(0.1);
  Async({
    pros::delay(100);
    rd4b::moveTarget(300);
    pros::delay(400);
    rd4b::moveTarget(0);
  }) pros::delay(1300);
  //rd4b::moveTarget(2000, 110); // was 1280
  //pros::delay(500);
  // drive::moveDistanceProfile(0.65);
  Async({
    drive::moveTo(Pose(2.6_ft, 0_ft, 0_deg), std::nullopt, std::nullopt,
                  std::nullopt, std::nullopt);
  }) 
  while (!drive::isAtTarget()) { 
    //printf("Still Waiting\n");
    pros::delay(10); 
  }
  drive::stop();

  Async({
    drive::moveTo(Pose(0.05_ft, -1.0_ft, -50_deg), std::nullopt, std::nullopt,
                  std::nullopt, std::nullopt);
  })
  while (!drive::isAtTarget()) { 
    //printf("Still Waiting\n");
    pros::delay(10);
  }
  drive::stop();

  Async({
    drive::moveTo(Pose(2.6_ft, -1.25_ft, -25_deg), std::nullopt, std::nullopt,
                  std::nullopt, std::nullopt);
  }) 
  while (!drive::isAtTarget()) { 
    //printf("Still Waiting\n");
    pros::delay(10); 
  }
  drive::stop();

}

void redFarAutoParkOnly() {}

// REMEMBER TO REVERSE
void skills() {}