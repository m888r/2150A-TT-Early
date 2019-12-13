#include "config.hpp"

#include "api.h"
#include "okapi/api.hpp"

namespace robot {
using namespace okapi::literals;
okapi::QLength wheelDiameter = 3.25_in * sqrt(2);
okapi::QLength encoderWheelDiameter = 2.75_in;
okapi::QLength encoderWidth = 10_in * (363.55/360.0);

okapi::Motor frontRight(-17);
okapi::Motor frontLeft(15);
okapi::Motor backRight(-14);
okapi::Motor backLeft(1);

okapi::Motor intakeRight(-20);
okapi::Motor intakeLeft(13);
okapi::MotorGroup intakeGroup({intakeRight, intakeLeft});

okapi::Motor tilt(-18);

okapi::Motor lift(19);

okapi::ADIEncoder leftEnc('A', 'B', true);
okapi::ADIEncoder centerEnc('C', 'D', true);
okapi::ADIEncoder rightEnc('G', 'H', true);
motion::Odometry odometry(rightEnc, leftEnc, centerEnc, encoderWidth,
                          encoderWheelDiameter, 6.75_in);

okapi::XDriveModel xDrive(std::make_shared<okapi::Motor>(frontLeft),
                          std::make_shared<okapi::Motor>(frontRight),
                          std::make_shared<okapi::Motor>(backRight),
                          std::make_shared<okapi::Motor>(backLeft),
                          std::make_shared<okapi::ADIEncoder>(leftEnc),
                          std::make_shared<okapi::ADIEncoder>(rightEnc), 200,
                          12000);

motion::XDrive driveKinematics(frontLeft, frontRight, backLeft, backRight, 0.86_mps, 13.5_in); // theoretical maxvel of each wheel is 0.86446157851 mps

void printController(void* p) {
  okapi::Controller master;
  master.clear();
  while (true) {
    std::string printStr =
       "A: " + std::to_string(odometry.getPose().heading.convert(okapi::degree));
    //std::string printStr = "C: " + std::to_string(centerEnc.get());
    master.setText(0, 0, printStr);
    pros::delay(51);

    std::string xStr = "X: " + std::to_string(odometry.getPose().position.getX().convert(okapi::foot));
    std::string encLStr = "L: " + std::to_string(leftEnc.get());
    master.setText(1, 0, xStr);
    pros::delay(51);
    std::string yStr = "Y: " + std::to_string(odometry.getPose().position.getY().convert(okapi::foot));
    std::string encRStr = "R: " + std::to_string(rightEnc.get());
    master.setText(2, 0, yStr);
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