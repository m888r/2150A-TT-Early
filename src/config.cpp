#include "config.hpp"

#include "api.h"
#include "okapi/api.hpp"

namespace robot {
using namespace okapi::literals;
okapi::QLength wheelDiameter = 3.255_in * sqrt(2);
okapi::QLength encoderWheelDiameter = 2.75_in;
okapi::QLength encoderWidth = 10_in * (363.55/360.0);

okapi::Motor frontRight(-17);
okapi::Motor frontLeft(13);
okapi::Motor backRight(-15);
okapi::Motor backLeft(12);

okapi::Motor intakeRight(-20);
okapi::Motor intakeLeft(1);
okapi::MotorGroup intakeGroup({intakeRight, intakeLeft});

okapi::Motor tilt(-18);

okapi::Motor lift(19);

okapi::ADIEncoder leftEnc('A', 'B', true);
okapi::ADIEncoder centerEnc('C', 'D');
okapi::ADIEncoder rightEnc('G', 'H', true);
motion::Odometry odometry(rightEnc, leftEnc, centerEnc, encoderWidth,
                          encoderWheelDiameter, 0_in);

okapi::XDriveModel xDrive(std::make_shared<okapi::Motor>(frontLeft),
                          std::make_shared<okapi::Motor>(frontRight),
                          std::make_shared<okapi::Motor>(backRight),
                          std::make_shared<okapi::Motor>(backLeft),
                          std::make_shared<okapi::ADIEncoder>(leftEnc),
                          std::make_shared<okapi::ADIEncoder>(rightEnc), 200,
                          12000);

void printController(void* p) {
  okapi::Controller master;
  master.clear();
  while (true) {
    std::string printStr =
        "A: " + std::to_string(odometry.getPose().heading.convert(okapi::degree));
    master.setText(0, 0, printStr);
    pros::delay(51);

    std::string encStr = "L: " + std::to_string(leftEnc.get());
    master.setText(1, 0, encStr);
    pros::delay(51);
    master.setText(2, 0, "R: " + std::to_string(rightEnc.get()));
    pros::delay(51);
  }
}

pros::Task bruhTask(printController, nullptr, "Controller Print");

// okapi::ChassisController chassisStraight =
// okapi::ChassisControllerBuilder.withMotors({frontLeft, frontRight},
// {backLeft, backRight}).withDimensions({3.25 * okapi::inch, 11 *
// okapi::inch}).build();

// okapi::AsyncMotionProfileController straightAMPC =
// okapi::AsyncMotionProfileControllerBuilder.withOutput().buildMotionProfileController();
}  // namespace robot