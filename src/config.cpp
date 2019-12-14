#include "config.hpp"

#include "api.h"
#include "okapi/api.hpp"

namespace robot {
using namespace okapi::literals;
okapi::QLength wheelDiameter = 3.25_in * sqrt(2);
okapi::QLength encoderWheelDiameter = 2.75_in;
okapi::QLength encoderWidth = 10_in * (363.55 / 360.0) * (90.0 / 93.26) * (95.8/90.0) * (90.5 / 89.3);

const int frontRightPort = 20;
const int frontLeftPort = 14;
const int backRightPort = 15;
const int backLeftPort = 1;

okapi::Motor frontRight(-frontRightPort);
okapi::Motor frontLeft(frontLeftPort);
okapi::Motor backRight(-backRightPort);
okapi::Motor backLeft(backLeftPort);

okapi::Motor frontLeftUV(-frontLeftPort);
okapi::Motor frontRightUV(-frontRightPort);
okapi::Motor backLeftUV(-backLeftPort);
okapi::Motor backRightUV(-backRightPort);

okapi::Motor intakeRight(-19);
okapi::Motor intakeLeft(13);
okapi::MotorGroup intakeGroup({intakeRight, intakeLeft});

okapi::Motor tilt(-17);

okapi::Motor lift(18);

// IMU port 16

okapi::ADIEncoder leftEnc('A', 'B', true);
okapi::ADIEncoder centerEnc('C', 'D');
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

motion::XDrive driveKinematics(
    frontLeftUV, frontRightUV, backLeftUV, backRightUV, 0.86_mps,
    16.3_in);  // theoretical maxvel of each wheel is 0.86446157851 mps

void printController(void* p) {
  okapi::Controller master;
  master.clear();
  while (true) {
    Eigen::Vector4d u;
    // seems like max angvel is 2.2 radps
    // but max linear vel seems like 0.95 mps which is strange
    u << frontLeftUV.getActualVelocity(), frontRightUV.getActualVelocity(),
        backRightUV.getActualVelocity(), backLeftUV.getActualVelocity();
    u = driveKinematics.angularWheelSpeedToLinear(3.25_in, u);
    Eigen::Vector3d rates = driveKinematics.fk(odometry.getStateVector(), u);

    std::string angVelString = "AV: " + std::to_string(rates(2));
    std::string xVel = "X: " + std::to_string(rates(0));
    std::string yVel = "Y: " + std::to_string(rates(1));
    
    std::string printStr =
        "A: " +
        std::to_string(odometry.getPose().heading.convert(okapi::degree));
    //std::string printStr = "C: " + std::to_string(centerEnc.get());
    master.setText(0, 0, printStr);
    pros::delay(51);

    std::string xStr =
        "X: " +
        std::to_string(odometry.getPose().position.getX().convert(okapi::foot));
    std::string encLStr = "L: " + std::to_string(leftEnc.get());
    master.setText(1, 0, xStr);
    //printf((xStr + ", ").c_str());
    pros::delay(51);
    std::string yStr =
        "Y: " +
        std::to_string(odometry.getPose().position.getY().convert(okapi::foot));
    std::string encRStr = "R: " + std::to_string(rightEnc.get());
    master.setText(2, 0, yStr);
    //printf((yStr + "\n").c_str());

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