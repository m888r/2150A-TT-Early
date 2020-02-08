#include "main.h"
#include "auton/autons.hpp"
//#include "auton/autos.hpp"

#include "subsystem/xdriveutilities.hpp"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  robot::intakeLeft.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
  robot::intakeRight.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);

  subsystem::intake::init();
  subsystem::tray::init();
  subsystem::rd4b::init();

  robot::odometry.reset();

  subsystem::drive::initXDriveDebug();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  robot::xDrive.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  robot::lift.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  pros::delay(200);
  auton::init();
  //lcd::initButtons();
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {

  //reset intake, rd4b, tilter
  robot::intakeGroup.tarePosition();
  robot::lift.tarePosition();
  robot::tilt.tarePosition();
  pros::delay(20);

  auton::runAuton();

  //robot::odometry.reset();
  //test();

  //lcd::runAuton();

  //redCloseAuto();
  //blueCloseAuto();
  //blueFarAuto();
  //redFarAuto();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  okapi::Controller master(okapi::ControllerId::master);

  robot::xDrive.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);

  subsystem::rd4b::changeState(subsystem::rd4b::state::manual);
  robot::lift.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  
  /// REMEMBER TO CHANGE TO MANUAL
  subsystem::intake::changeState(subsystem::intake::state::manual);
  robot::intakeGroup.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);

  okapi::ControllerButton placeButton(okapi::ControllerDigital::up);
  okapi::ControllerButton standbyButton(okapi::ControllerDigital::left);

  okapi::ControllerButton resetTrayButton(okapi::ControllerDigital::down);
  okapi::ControllerButton defenseButton(okapi::ControllerDigital::B);


  okapi::ControllerButton motorTestBtn(okapi::ControllerDigital::A);
  
  bool defenseModeActive = false;
  bool placingModeActive = false;

	while (true) {
		double strafeLeft = master.getAnalog(okapi::ControllerAnalog::leftX);
		double forwardLeft = master.getAnalog(okapi::ControllerAnalog::leftY);
		double strafeRight = master.getAnalog(okapi::ControllerAnalog::rightX);
    double forwardRight = master.getAnalog(okapi::ControllerAnalog::rightY);

    double strafe = master.getAnalog(okapi::ControllerAnalog::leftX);
    double forward = master.getAnalog(okapi::ControllerAnalog::leftY);
    double turn = master.getAnalog(okapi::ControllerAnalog::rightX);
    int threshold = 0.09;

    if (defenseButton.changedToPressed()) {
      defenseModeActive = !defenseModeActive;
    }

    if (fabs(strafeLeft) > threshold || fabs(strafeRight) > threshold || fabs(forwardLeft) > threshold || fabs(forwardRight) > threshold) {
      robot::xDrive.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);

      robot::frontRight.moveVoltage((forwardRight - strafeRight) * 12000.0);
      robot::frontLeft.moveVoltage((forwardLeft + strafeLeft) * 12000.0);
      robot::backRight.moveVoltage((forwardRight + strafeRight) * 12000.0);
      robot::backLeft.moveVoltage((forwardLeft - strafeLeft) * 12000.0);
    } else {
      
      robot::xDrive.setBrakeMode(defenseModeActive ? okapi::AbstractMotor::brakeMode::hold : okapi::AbstractMotor::brakeMode::coast);

      robot::frontRight.moveVelocity(0);
      // if (motorTestBtn.isPressed()) {
      //   robot::frontRight.moveVoltage(12000);
      // } else {
      //   robot::frontRight.moveVoltage(0);
      // }

      robot::frontLeft.moveVelocity(0);
      robot::backRight.moveVelocity(0);
      robot::backLeft.moveVelocity(0);
    }

    // if (motorTestBtn.isPressed()) {
    //   robot::intakeRight.moveVoltage(12000);
    // } else {
    //   robot::intakeRight.moveVoltage(0);
    // }

    if (placingModeActive) {
      if (placeButton.isPressed()) {
        subsystem::tray::changeMode(subsystem::tray::mode::placing);
      } else {
        subsystem::tray::changeMode(subsystem::tray::mode::holding);
      }
    }

    if (placeButton.changedToPressed()) {
      if (subsystem::tray::currMode == subsystem::tray::mode::prepared || placingModeActive) {
        placingModeActive = true;
      } else {
        subsystem::tray::changeMode(subsystem::tray::mode::prepared);
      }
    }
    
    if (standbyButton.changedToPressed()) {
      placingModeActive = false;
      subsystem::tray::changeMode(subsystem::tray::mode::standby);
    }

    if (resetTrayButton.changedToPressed()) {
      placingModeActive = false;
      subsystem::tray::changeMode(subsystem::tray::mode::resetting);
    }

    pros::delay(10);
	}
}
