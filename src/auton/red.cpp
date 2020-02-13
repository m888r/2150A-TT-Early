#include "auton/autons.hpp"

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

namespace auton {
  void redProtectedEight() {
    okapi::Timer timer;
    robot::lift.tarePosition();
    odometry.reset();
    pros::delay(50);
    
    Async({
      tray::changeMode(tray::mode::prepared);
      rd4b::changeState(rd4b::state::up);
      pros::delay(700);
      //rd4b::moveTarget(0);
      tray::changeMode(tray::mode::standby);
      rd4b::changeState(rd4b::state::resetting);
      intake::in();
    })
    drive::moveDistanceProfile(0.125);
    pros::delay(250);
    rd4b::moveTarget(1250, 200);  // was 1280
    timer.placeMark();
    while (std::abs(robot::lift.getPosition()) < 600 &&
          timer.getDtFromMark() < 1500_ms) {
      pros::delay(10);
    }
    // drive::moveDistanceProfile(0.65);
    Async({
      drive::moveTo(Pose(2.41_ft, 0_ft, 0_deg), std::nullopt, std::nullopt, // was 2.35
                    std::nullopt, std::nullopt);
    }) while (!drive::isAtTarget()) {
      // printf("Still Waiting\n");
      pros::delay(10);
    }
    drive::stop();
    rd4b::moveTarget(0, 200);
    timer.placeMark();
    while (std::abs(robot::lift.getPosition()) > 700 &&
          timer.getDtFromMark() < 1500_ms) {
      pros::delay(10);
    }

    rd4b::changeState(rd4b::state::resetting);
    pros::delay(50);
    timer.placeMark();
    while (rd4b::getState() != rd4b::state::holding &&
          timer.getDtFromMark() < 1500_ms) {
      pros::delay(10);
    }
    tray::changeMode(tray::mode::standby);
    Async({
      drive::moveTo(Pose(3.9_ft, 0.4_ft, 30_deg), 2.4_in, std::nullopt, std::nullopt, std::nullopt, PIDGains(4.6, 0, 0), PIDGains(1.5, 0, 0)); // position 4
    }) // was 3.9, 0.3, 40 degrees
    while (!drive::isAtTarget()) {
      // printf("Still Waiting\n");
      pros::delay(10);
    }
    drive::stop();
    // tray::changeMode(tray::mode::spinnable);
    rd4b::changeState(rd4b::state::resetting);
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
    //pros::delay(350);
    intake::changeState(intake::state::outPosition);
    
    while (!drive::isAtTarget() && timer.getDtFromMark() < 2200_ms) {
      // printf("Still Waiting\n");
      pros::delay(10);
    }
    drive::stop();
    
    // robot::intakeGroup.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
    //intake::changeState(intake::state::free);
    rd4b::changeState(rd4b::state::placing);
    tray::changeMode(tray::mode::placing);

    timer.placeMark();
    while (tray::currMode != tray::mode::holding &&
          timer.getDtFromMark() < 1500_ms) {
      pros::delay(10);
    }
    //intake::changeState(intake::state::free);
    robot::intakeGroup.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
    rd4b::moveTarget(300, 200); // was 500
    //pros::delay(600);
    Async({
      pros::delay(600);
      tray::changeMode(tray::mode::standby);
    })
    drive::moveDistanceProfile(-0.5, 0_deg, 0.8); // drive away from stack semi slowly
    rd4b::changeState(rd4b::state::resetting);
    //pros::delay(1000);

  }

  void redUnprotectedNine() {
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
}