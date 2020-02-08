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
  void blueProtectedEight() {
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

  void blueUnprotectedNine() {
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
}