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
  okapi::Timer printTimer;
  pros::delay(50);
  Async({
    drive::moveTo(Pose(4_ft, 0_ft, 180_deg), std::nullopt, std::nullopt,
                  std::nullopt, std::nullopt);
  })
  while (!drive::isAtTarget()) {
    if (printTimer.repeat(100_ms)) {
      printf("Still Waiting\n");
    }
    pros::delay(10);
  }
  drive::stop();
  printf("Stopped\n");
  Async({ drive::moveTo(Pose(0_ft, 0_ft, 0_deg));
  })
  while (!drive::isAtTarget()) {
    if (printTimer.repeat(100_ms)) {
      printf("Waiting for second movement");
    }
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
  okapi::Timer timer;
  robot::lift.tarePosition();
  odometry.reset();
  pros::delay(50);
  intake::in();
  Async({
    pros::delay(100);
    rd4b::moveTarget(300);
    pros::delay(400);
    rd4b::moveTarget(0);   
    rd4b::changeState(rd4b::state::resetting);
  })
  drive::moveDistanceProfile(0.2);
  //pros::delay(500);
  rd4b::moveTarget(1250, 200);  // was 1280
  timer.placeMark();
  while (std::abs(robot::lift.getPosition()) < 300 &&
         timer.getDtFromMark() < 1500_ms) {
    pros::delay(10);
  }
  // drive::moveDistanceProfile(0.65);
  Async({
    drive::moveTo(Pose(2.35_ft, -0_ft, -0_deg), std::nullopt, std::nullopt,
                  std::nullopt, std::nullopt);
  }) while (!drive::isAtTarget()) {
    // printf("Still Waiting\n");
    pros::delay(10);
  }
  drive::stop();
  //rd4b::moveTarget(0);
  rd4b::changeState(rd4b::state::resetting);
  pros::delay(50);
  timer.placeMark();
  while (rd4b::getState() != rd4b::state::holding &&
         timer.getDtFromMark() < 3000_ms) {
    pros::delay(10);
  }
  tray::changeMode(tray::mode::standby);
  Async({
    drive::moveTo(Pose(3.9_ft,- 0.3_ft, -40_deg), std::nullopt, std::nullopt, std::nullopt, std::nullopt, PIDGains(4.6, 0, 0), PIDGains(1.4, 0, 0)); // position 4
  }) // was 3.9, 0.3, 40 degrees
  while (!drive::isAtTarget()) {
    // printf("Still Waiting\n");
    pros::delay(10);
  }
  drive::stop();
  // tray::changeMode(tray::mode::spinnable);
  Async({
    drive::moveTo(Pose(1.5_ft, -1.72_ft, -135_deg), std::nullopt, std::nullopt, std::nullopt, std::nullopt, PIDGains(4.6, 0, 0), PIDGains(1.4, 0, 0)); // position 5
  })
  drive::waitUntilSettled();
  pros::delay(300);
  printf("Got the cube, boutta prepare tray\n");
  intake::changeState(intake::state::free);
  tray::changeMode(tray::mode::prepared);
  pros::delay(350);
  // okapi::QAngle placeAngle = 110_deg;
  // drive::turnTo(placeAngle);

  Async({
    drive::moveTo(Pose(0.125_ft, -1.97_ft, -98_deg), 2.4_in, std::nullopt,
                  std::nullopt, std::nullopt, PIDGains(4.7, 0, 0), PIDGains(1.3, 0, 0)); // position 6, place stack
  })
  while (!drive::isAtTarget()) {
    // printf("Still Waiting\n");
    pros::delay(10);
  }
  drive::stop();
  
  robot::intakeGroup.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  intake::manual();
  tray::changeMode(tray::mode::placing);

  timer.placeMark();
  while (tray::currMode != tray::mode::holding &&
         timer.getDtFromMark() < 2000_ms) {
    pros::delay(10);
  }
  rd4b::moveTarget(500, 200);
  pros::delay(500);
  drive::moveDistanceProfile(-0.5, 0_deg, 0.8); // drive away from stack semi slowly
  tray::changeMode(tray::mode::standby);
  rd4b::changeState(rd4b::state::resetting);
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
  // rd4b::moveTarget(2000, 110); // was 1280
  // pros::delay(500);
  // drive::moveDistanceProfile(0.65);
  Async({
    drive::moveTo(Pose(2.6_ft, 0_ft, 0_deg), std::nullopt, std::nullopt,
                  std::nullopt, std::nullopt);
  }) while (!drive::isAtTarget()) {
    // printf("Still Waiting\n");
    pros::delay(10);
  }
  drive::stop();

  Async({
    drive::moveTo(Pose(0.05_ft, 1.0_ft, 50_deg), std::nullopt, std::nullopt,
                  std::nullopt, std::nullopt);
  }) while (!drive::isAtTarget()) {
    // printf("Still Waiting\n");
    pros::delay(10);
  }
  drive::stop();

  Async({
    drive::moveTo(Pose(2.6_ft, 1.25_ft, 25_deg), std::nullopt, std::nullopt,
                  std::nullopt, std::nullopt);
  }) while (!drive::isAtTarget()) {
    // printf("Still Waiting\n");
    pros::delay(10);
  }
  drive::stop();
}

// redclose
void redClosePaths() {}


// https://www.desmos.com/calculator/ubm9hfkpa7
void redCloseAuto() {
  okapi::Timer timer;
  robot::lift.tarePosition();
  odometry.reset();
  pros::delay(50);
  intake::in();
  Async({
    pros::delay(100);
    rd4b::moveTarget(300);
    pros::delay(400);
    rd4b::moveTarget(0);   
    rd4b::changeState(rd4b::state::resetting);
  })
  drive::moveDistanceProfile(0.2);
  //pros::delay(500);
  rd4b::moveTarget(1250, 200);  // was 1280
  timer.placeMark();
  while (std::abs(robot::lift.getPosition()) < 300 &&
         timer.getDtFromMark() < 1500_ms) {
    pros::delay(10);
  }
  // drive::moveDistanceProfile(0.65);
  Async({
    drive::moveTo(Pose(2.35_ft, 0_ft, 0_deg), std::nullopt, std::nullopt,
                  std::nullopt, std::nullopt);
  }) while (!drive::isAtTarget()) {
    // printf("Still Waiting\n");
    pros::delay(10);
  }
  drive::stop();
  //rd4b::moveTarget(0);
  rd4b::changeState(rd4b::state::resetting);
  pros::delay(50);
  timer.placeMark();
  while (rd4b::getState() != rd4b::state::holding &&
         timer.getDtFromMark() < 3000_ms) {
    pros::delay(10);
  }
  tray::changeMode(tray::mode::standby);
  Async({
    drive::moveTo(Pose(3.9_ft, 0.4_ft, 40_deg), 2.4_in, std::nullopt, std::nullopt, std::nullopt, PIDGains(4.6, 0, 0), PIDGains(1.5, 0, 0)); // position 4
  }) // was 3.9, 0.3, 40 degrees
  while (!drive::isAtTarget()) {
    // printf("Still Waiting\n");
    pros::delay(10);
  }
  drive::stop();
  // tray::changeMode(tray::mode::spinnable);
  Async({
    drive::moveTo(Pose(1.5_ft, 1.72_ft, 135_deg), 2.4_in, std::nullopt, std::nullopt, std::nullopt, PIDGains(4.6, 0, 0), PIDGains(1.4, 0, 0)); // position 5
  })
  drive::waitUntilSettled();
  pros::delay(300);
  printf("Got the cube, boutta prepare tray\n");
  //intake::changeState(intake::state::free);
  
  tray::changeMode(tray::mode::prepared);
  //pros::delay(350);
  // okapi::QAngle placeAngle = 110_deg;
  // drive::turnTo(placeAngle);

  Async({
    drive::moveTo(Pose(0.0_ft, 1.87_ft, 98_deg), 2.4_in, std::nullopt,
                  std::nullopt, std::nullopt, PIDGains(4.7, 0, 0), PIDGains(1.3, 0, 0)); // position 6, place stack
  })
  timer.placeMark();
  Async({
    pros::delay(350);
    intake::changeState(intake::state::outPosition);
  })
  while (!drive::isAtTarget() && timer.getDtFromMark() < 2000_ms) {
    // printf("Still Waiting\n");
    pros::delay(10);
  }
  drive::stop();
  
  robot::intakeGroup.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
  //intake::changeState(intake::state::free);
  tray::changeMode(tray::mode::placing);

  timer.placeMark();
  while (tray::currMode != tray::mode::holding &&
         timer.getDtFromMark() < 1400_ms) {
    pros::delay(10);
  }
  intake::changeState(intake::state::free);
  robot::intakeGroup.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  rd4b::moveTarget(500, 200); // was 500
  pros::delay(500);
  Async({
    pros::delay(300);
    tray::changeMode(tray::mode::standby);
  })
  drive::moveDistanceProfile(-0.5, 0_deg, 0.8); // drive away from stack semi slowly
  rd4b::changeState(rd4b::state::resetting);
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
  // rd4b::moveTarget(2000, 110); // was 1280
  // pros::delay(500);
  // drive::moveDistanceProfile(0.65);
  Async({
    drive::moveTo(Pose(2.6_ft, 0_ft, 0_deg), std::nullopt, std::nullopt,
                  std::nullopt, std::nullopt);
  }) while (!drive::isAtTarget()) {
    // printf("Still Waiting\n");
    pros::delay(10);
  }
  drive::stop();

  Async({
    drive::moveTo(Pose(0.05_ft, -1.0_ft, -50_deg), std::nullopt, std::nullopt,
                  std::nullopt, std::nullopt);
  }) while (!drive::isAtTarget()) {
    // printf("Still Waiting\n");
    pros::delay(10);
  }
  drive::stop();

  Async({
    drive::moveTo(Pose(2.6_ft, -1.25_ft, -25_deg), std::nullopt, std::nullopt,
                  std::nullopt, std::nullopt);
  }) while (!drive::isAtTarget()) {
    // printf("Still Waiting\n");
    pros::delay(10);
  }
  drive::stop();
}

void redFarAutoParkOnly() {}

// start on red side in line with 3 cube line
void skills() {
  okapi::Timer timer;
  robot::lift.tarePosition();
  odometry.setPose(Pose(8_in, 4.25_ft, 0_deg));
  pros::delay(50);

  // deploy things
  intake::in();
  // Async({
  //   pros::delay(100);
  //   rd4b::moveTarget(300);
  //   pros::delay(400);
  //   rd4b::moveTarget(0);   
  //   rd4b::changeState(rd4b::state::resetting);
  // })

  // // grab cube for tower, though this is unnecessary
  // Async({
  //   drive::moveTo(Pose(1.5_ft, 5_ft, 45_deg), 2.4_in);
  // })
  // drive::waitUntilSettled();
  // lift to drop it
  rd4b::moveTarget(1400, 200);
  timer.placeMark();
  while (std::abs(robot::lift.getPosition()) < 600 &&
         timer.getDtFromMark() < 1500_ms) {
    pros::delay(10);
  }
  
  intake::changeState(intake::state::outPosition);

  // drive forward to deposit cube
  Async({
    drive::moveTo(Pose(1.5_ft, 5.5_ft, 45_deg), 2.4_in);
  })
  drive::waitUntilSettled();
  intake::outAtSpeed(6000);
  pros::delay(500);

  Async({
    drive::moveTo(Pose(3.25_ft, 4.25_ft, 0_deg), 2.4_in, 2.5_deg);
  })
  pros::delay(300);
  intake::in();
  rd4b::changeState(rd4b::state::resetting);
  drive::waitUntilSettled();

  //wait for it to finish going down
  while (std::abs(robot::lift.getPosition()) > 200 &&
         timer.getDtFromMark() < 1500_ms) {
    pros::delay(10);
  }

}