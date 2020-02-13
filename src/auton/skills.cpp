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

  // start on red unprotected corner
  void progSkills91() {
    okapi::Timer timer;
    robot::lift.tarePosition();
    odometry.setPose(Pose(7.2_in, 4.25_ft, 0_deg));
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
    intake::outAtSpeed(8000);
    pros::delay(300);

    // drive to the line cubes (might want to consider doing two targets with wusnostop instead of just one)
    Async({
      drive::moveTo(Pose(3.25_ft, 4.25_ft, 0_deg), 2.4_in, 2.5_deg);
    })
    pros::delay(300);
    intake::in();
    rd4b::changeState(rd4b::state::resetting);

    //wait for it to finish going down
    // while (std::abs(robot::lift.getPosition()) > 800 &&
    //       timer.getDtFromMark() < 1500_ms) {
    //   pros::delay(10);
    // }
    drive::waitUntilSettled();

    rd4b::moveTarget(450, 200);
    timer.placeMark();
    while (std::abs(robot::lift.getPosition()) < 350 &&
          timer.getDtFromMark() < 1500_ms) {
      pros::delay(10);
    }

    // continue to grab stacked cubes
    Async({
      drive::moveTo(Pose(3.75_ft, 4.25_ft, 0_deg), 2.4_in);
    })
    drive::waitUntilSettled();

    // lower lift to grab them
    rd4b::changeState(rd4b::state::resetting);
    timer.placeMark();
    while (std::abs(robot::lift.getPosition()) > 50 &&
          timer.getDtFromMark() < 1500_ms) {
      pros::delay(10);
    }
    //pros::delay(300); // wait to make sure last cube grabbed

    // drive to the orange cube in front of the tower
    Async({
      drive::moveTo(Pose(5_ft, 3.5_ft, -45_deg), 2.4_in);
    })
    drive::waitUntilSettled();

    // drive to tower while lifting
    intake::changeState(intake::state::outPosition);
    rd4b::moveTarget(1050, 200); // 1050 small tower/alliance tower height
    timer.placeMark();
    while (std::abs(robot::lift.getPosition()) < 800 &&
          timer.getDtFromMark() < 1500_ms) {
      pros::delay(10);
    }
    Async({
      drive::moveTo(Pose(5.5_ft, 4_ft, 0_deg), 2.4_in);
    })
    drive::waitUntilSettled();
    intake::outAtSpeed(8000);
    pros::delay(300);
  }
}
