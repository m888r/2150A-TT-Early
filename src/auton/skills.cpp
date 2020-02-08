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
  void progSkills91() {
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
}
