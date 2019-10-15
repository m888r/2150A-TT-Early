#include "main.h"

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

  lcd::initButtons();
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
  lcd::runAuton();
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

  robot::backLeft.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  robot::backRight.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  robot::frontLeft.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
  robot::frontRight.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);

  subsystem::rd4b::changeState(subsystem::rd4b::state::manual);
  subsystem::intake::manual();

  okapi::ControllerButton placeButton(okapi::ControllerDigital::up);
  okapi::ControllerButton standbyButton(okapi::ControllerDigital::down);

	while (true) {
		double strafeLeft = master.getAnalog(okapi::ControllerAnalog::leftX);
		double forwardLeft = master.getAnalog(okapi::ControllerAnalog::leftY);
		double strafeRight = master.getAnalog(okapi::ControllerAnalog::rightX);
    double forwardRight = master.getAnalog(okapi::ControllerAnalog::rightY);

    double strafe = master.getAnalog(okapi::ControllerAnalog::leftX);
    double forward = master.getAnalog(okapi::ControllerAnalog::leftY);
    double turn = master.getAnalog(okapi::ControllerAnalog::rightX);
    int threshold = 0.05;

    if (fabs(strafeLeft) > threshold || fabs(strafeRight) > threshold || fabs(forwardLeft) > threshold || fabs(forwardRight) > threshold) {
      robot::backLeft.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
      robot::backRight.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
      robot::frontLeft.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);
      robot::frontRight.setBrakeMode(okapi::AbstractMotor::brakeMode::coast);

      robot::frontRight.moveVoltage((forwardRight - strafeRight) * 12000.0);
      robot::frontLeft.moveVoltage((forwardLeft + strafeLeft) * 12000.0);
      robot::backRight.moveVoltage((forwardRight + strafeRight) * 12000.0);
      robot::backLeft.moveVoltage((forwardLeft - strafeLeft) * 12000.0);
    } else {
      robot::backLeft.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
      robot::backRight.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
      robot::frontLeft.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
      robot::frontRight.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);

      robot::frontRight.moveVelocity(0);
      robot::frontLeft.moveVelocity(0);
      robot::backRight.moveVelocity(0);
      robot::backLeft.moveVelocity(0);
    }

    // double intakePower = (master.getDigital(okapi::ControllerDigital::R1) - master.getDigital(okapi::ControllerDigital::R2)) * 12000.0;
    // if (fabs(intakePower) >= 200) {
    //   robot::intakeLeft.moveVoltage(intakePower);
    //   robot::intakeRight.moveVoltage(intakePower);
    // } else {
    //   robot::intakeLeft.moveVelocity(0);
    //   robot::intakeRight.moveVelocity(0);
    // }

    if (placeButton.changedToPressed()) {
      subsystem::tray::changeMode(subsystem::tray::mode::placing);
    }
    
    if (standbyButton.changedToPressed()) {
      subsystem::tray::changeMode(subsystem::tray::mode::standby);
    }


    pros::delay(10);
	}
}
