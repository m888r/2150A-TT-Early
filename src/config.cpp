#include "config.hpp"

#include "api.h"
#include "okapi/api.hpp"

namespace robot {
using namespace okapi::literals;
okapi::QLength wheelDiameter = 4.125_in;
okapi::QLength encoderWheelDiameter = 2.75_in;
okapi::QLength encoderWidth = 10_in;

okapi::Motor frontRight(-20);
okapi::Motor frontLeft(10);
okapi::Motor backRight(-11);
okapi::Motor backLeft(2);

okapi::Motor intakeRight(17);
okapi::Motor intakeLeft(-7);
okapi::MotorGroup intakeGroup({intakeRight, intakeLeft});

okapi::Motor tilt(14);

okapi::Motor lift(13);

okapi::ADIEncoder rightEnc('A', 'B');
okapi::ADIEncoder leftEnc('C', 'D', true);
okapi::ADIEncoder centerEnc('E', 'F');
motion::Odometry odometry(rightEnc, leftEnc, centerEnc, encoderWidth,
                          encoderWheelDiameter, 0_in);

okapi::XDriveModel xDrive(std::make_shared<okapi::Motor>(frontLeft),
                          std::make_shared<okapi::Motor>(frontRight),
                          std::make_shared<okapi::Motor>(backRight),
                          std::make_shared<okapi::Motor>(backLeft),
                          std::make_shared<okapi::ADIEncoder>(leftEnc),
                          std::make_shared<okapi::ADIEncoder>(rightEnc), 200,
                          12000);
// okapi::ChassisController chassisStraight =
// okapi::ChassisControllerBuilder.withMotors({frontLeft, frontRight},
// {backLeft, backRight}).withDimensions({3.25 * okapi::inch, 11 *
// okapi::inch}).build();

// okapi::AsyncMotionProfileController straightAMPC =
// okapi::AsyncMotionProfileControllerBuilder.withOutput().buildMotionProfileController();
}  // namespace robot