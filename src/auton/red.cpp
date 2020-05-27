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
      pros::delay(400);
      //rd4b::moveTarget(0);
      tray::changeMode(tray::mode::standby);
      rd4b::changeState(rd4b::state::resetting);
      intake::in();
    })
    // Async({ // THIS ALWAYS CAUSES A SLIGHT DELAY
    //   drive::moveTo(Pose(0.5_ft, 0_ft, 0_deg), std::nullopt, std::nullopt, // was 2.35
    //                 std::nullopt, std::nullopt, PIDGains(5.0, 0, 0));
    // }) while (!drive::isAtTarget()) {
    //   // printf("Still Waiting\n");
    //   pros::delay(10);
    // }
    //drive::stop();
    drive::moveDistanceProfile(0.125);
    timer.placeMark();
    while (std::abs(robot::lift.getPosition()) > 100 &&
          timer.getDtFromMark() < 750_ms) {
      pros::delay(10);
    }
    rd4b::moveTarget(1300, 200);  // was 1250
    timer.placeMark();
    while (std::abs(robot::lift.getPosition()) < 800 &&
          timer.getDtFromMark() < 2500_ms) {
      pros::delay(10);
    }
    // drive::moveDistanceProfile(0.65);
    Async({
      drive::moveTo(Pose(2.4_ft, 0_ft, 0_deg), std::nullopt, std::nullopt, // was 2.35
                    std::nullopt, std::nullopt, PIDGains(4.05, 0, 0), PIDGains(1.3, 0.0, 0.0), 1.4);
    }) while (!drive::isAtTarget()) {
      // printf("Still Waiting\n");
      pros::delay(10);
    }
    drive::stop();
    rd4b::moveTarget(0, 100);
    timer.placeMark();

    // wait till the lift lowers a bit and then do prepared mode to minimize jerk
    while (std::abs(robot::lift.getPosition()) > 1250 &&
          timer.getDtFromMark() < 1500_ms) {
      pros::delay(10);
    }
    tray::changeMode(tray::mode::prepared);

    rd4b::changeState(rd4b::state::resetting);
    pros::delay(50);
    timer.placeMark();
    while (rd4b::getState() != rd4b::state::holding &&
          timer.getDtFromMark() < 1500_ms) {
      pros::delay(10);
    }
    tray::changeMode(tray::mode::standby);
    Async({
      drive::moveTo(Pose(3.9_ft, 0.4_ft, 30_deg), 4_in, 3_deg, std::nullopt, std::nullopt, PIDGains(5.3, 0, 0), PIDGains(1.5, 0, 0), std::nullopt, 0.16); // position 4
    }) // was 3.9, 0.3, 40 degrees
    while (!drive::isAtTarget()) {
      // printf("Still Waiting\n");
      pros::delay(10);
    }
    drive::stop();
    // tray::changeMode(tray::mode::spinnable);
    rd4b::changeState(rd4b::state::resetting);
    Async({
      drive::moveTo(Pose(1.5_ft, 1.72_ft, 135_deg), 5_in, 2_deg, std::nullopt, std::nullopt, PIDGains(5.3, 0, 0), PIDGains(1.4, 0, 0), std::nullopt, 0.16); // position 5
    })
    drive::waitUntilSettled();
    pros::delay(300);
    printf("Got the cube, boutta prepare tray\n");
    //intake::changeState(intake::state::free);
    
    tray::changeMode(tray::mode::prepared);
    //pros::delay(350);
    // okapi::QAngle placeAngle = 110_deg;
    // drive::turnTo(placeAngle);
    
    rd4b::changeState(rd4b::state::resetting);
    
    intake::changeState(intake::state::manual);

    Async({
      drive::moveTo(Pose(-0.35_ft, 1.87_ft, 90_deg), 3.5_in, std::nullopt,
                    std::nullopt, std::nullopt, PIDGains(4.75, 0, 0), PIDGains(1.3, 0, 0), 1.0, 0.1); // position 6, place stack
    })
    timer.placeMark();
    //pros::delay(350);
    // Async({
    //   pros::delay(200);
      
    //   intake::outAtSpeed(1000); 
    // })

    // pros::delay(300);

    // while (drive::getDistanceToTarget() > 8_in && timer.getDtFromMark() < 2500_ms) {
    //   pros::delay(10);
    // }

    // robot::intakeGroup.moveVoltage(-4000);
    
    while (!drive::isAtTarget() && timer.getDtFromMark() < 3500_ms) {
      // printf("Still Waiting\n");
      pros::delay(10);
    }
    drive::stop();
    pros::delay(600);
    
    intake::changeState(intake::state::manual);

    // robot::intakeGroup.moveVoltage(-3000);
    // robot::intakeGroup.setCurrentLimit(1000);
    // pros::delay(1000);
    // robot::intakeGroup.moveVoltage(0);
    // robot::intakeGroup.setCurrentLimit(2500);
    rd4b::changeState(rd4b::state::placing);
    tray::changeMode(tray::mode::placing);

    timer.placeMark();
    while (tray::currMode != tray::mode::holding &&
          timer.getDtFromMark() < 2500_ms) {
      pros::delay(10);
    }
    //pros::delay(200);
    //intake::changeState(intake::state::free);
    //pros::delay(600);
    Async({
      robot::intakeGroup.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
      //rd4b::moveTarget(500, 200); // was 500
      intake::changeState(intake::state::manual);
      pros::delay(600);
      tray::changeMode(tray::mode::standby);
    })
    //pros::delay(200);
    drive::moveDistanceProfile(-0.5, 0_deg, 0.8); // drive away from stack semi slowly
    rd4b::changeState(rd4b::state::resetting);    //pros::delay(1000);

  }

  void redUnprotectedNine() {
    okapi::Timer timer;
    robot::lift.tarePosition();
    odometry.reset();
    pros::delay(50);
    intake::in();
    Async({
      tray::changeMode(tray::mode::prepared);
      rd4b::changeState(rd4b::state::up);
      pros::delay(400);
      //rd4b::moveTarget(0);
      tray::changeMode(tray::mode::standby);
      rd4b::changeState(rd4b::state::resetting);
      intake::in();
    })
    
    drive::moveDistanceProfile(0.1);
    pros::delay(500);
    // rd4b::moveTarget(2000, 110); // was 1280
    // pros::delay(500);
    // drive::moveDistanceProfile(0.65);
    Async({
      drive::moveTo(Pose(2_ft, 0_ft, 0_deg), std::nullopt, std::nullopt,
                    std::nullopt, std::nullopt, PIDGains(5.3, 0, 0), PIDGains(1.3, 0, 0), 0.65);
    })
    drive::waitUntilSettled();

    rd4b::moveTarget(800, 200);
    pros::delay(100);
    rd4b::waitUntilSettled();

    // drive forward to grab the cubes, lift down while doing it
    Async({
      pros::delay(500);
      rd4b::changeState(rd4b::state::resetting);
    })
    
    drive::moveDistanceProfile(0.15);

    // drive backwards and away in order to get ready to grab another row of cubes
    Async({
      drive::moveTo(Pose(0.25_ft, -1.7_ft, -35_deg), std::nullopt, std::nullopt,
                    std::nullopt, std::nullopt, PIDGains(5.3, 0, 0), PIDGains(1.3, 0, 0));
    })
    drive::waitUntilSettled();

    Async({
      drive::moveTo(Pose(3_ft, -2_ft, 0_deg), 3_in, 3_deg,
                    std::nullopt, std::nullopt, PIDGains(5.3, 0, 0), PIDGains(1.3, 0, 0), 0.55, 0.005);
    })
    drive::waitUntilSettled();

    // has 9 cubes, now going to go place
    tray::changeMode(tray::mode::prepared);
    Async({
      drive::moveTo(Pose(0.02_ft, -2.7_ft, -135_deg), 3_in, std::nullopt,
                    std::nullopt, std::nullopt, PIDGains(5.3, 0, 0), PIDGains(1.3, 0, 0), 1.0, 0.1);
    })
    timer.placeMark();
    while (!drive::isAtTarget() && timer.getDtFromMark() < 5000_ms) {
      // printf("Still Waiting\n"); remember to add velocity timeout
      pros::delay(10);
    }
    if (timer.getDtFromMark() > 5000_ms) {
      printf("Timeout hit \n");
    }
    drive::stop();  

    timer.placeMark();
    // turn off intake state machine
    intake::changeState(intake::state::manual);

    // robot::intakeGroup.moveVoltage(-3000);
    // robot::intakeGroup.setCurrentLimit(1000);
    // pros::delay(1000);
    // robot::intakeGroup.moveVoltage(0);
    // robot::intakeGroup.setCurrentLimit(2500);
    rd4b::changeState(rd4b::state::placing);
    tray::changeMode(tray::mode::placing);

    timer.placeMark();
    while (tray::currMode != tray::mode::holding &&
          timer.getDtFromMark() < 2500_ms) {
      pros::delay(10);
    }
    pros::delay(200);
    //intake::changeState(intake::state::free);
    robot::intakeGroup.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
    rd4b::moveTarget(500, 200); // was 500
    intake::changeState(intake::state::manual);
    //pros::delay(200);
    Async({
      pros::delay(600);
      tray::changeMode(tray::mode::standby);
    })
    //pros::delay(200);
    drive::moveDistanceProfile(-0.5, -135_deg, 0.8); // drive away from stack semi slowly
    rd4b::changeState(rd4b::state::resetting);    //pros::delay(1000);
  }
}