#include "motion/xdrive.hpp"
#include "Eigen/Core"
//#include "Eigen/LU"
#include "Eigen/MatrixFunctions"

namespace motion {
XDrive::XDrive(okapi::Motor &leftFront, okapi::Motor &rightFront,
               okapi::Motor &leftBack, okapi::Motor &rightBack,
               okapi::QSpeed vMax, okapi::QLength b)
    : leftFront(leftFront),
      rightFront(rightFront),
      leftBack(leftBack),
      rightBack(rightBack),
      vMax(vMax.convert(okapi::mps)),
      b(b.convert(okapi::meter)) {}

double XDrive::calcVelLimit(double theta, double yawRate) {
  double base =
      fabs(vMax * std::sqrt(2) / fabs(std::sin(theta)) + fabs(std::cos(theta)));
  double scaled = base * fabs((maxYawRate() - yawRate) / maxYawRate());
  return scaled;
}

double XDrive::maxYawRate() {
  auto x = fk(Eigen::Vector3d(0.0, 0.0, 0.0),
              Eigen::Vector4d(vMax, vMax, vMax, vMax));

  return x(2);
}

Eigen::Vector3d XDrive::fk(Eigen::Vector3d x, Eigen::Vector4d u) {
  // if (velocityCapped) {
  //   u = u.cwiseMin(-vMax).cwiseMax(vMax);
  // }

  double theta = x(2);

  Eigen::Matrix3d A;
  Eigen::Matrix3d B;
  Eigen::Matrix<double, 3, 4> C;

  A << std::cos(theta), -std::sin(theta), 0,
       std::sin(theta), std::cos(theta), 0,
       0, 0, 1;

  double ootsqt = 1.0 / (2.0 * std::sqrt(2));

  B << ootsqt, ootsqt, 0,
       -ootsqt, ootsqt, 0,
       0, 0, 1.0 / b;

  C << 1, 0, -1, 
       0, 0, -1, 
       0, 1, 0.25, 
       0.25, 0.25, 0.25;

  return A * B * C * u;
}

Eigen::Vector4d XDrive::ik(Eigen::Vector3d x, Eigen::Vector3d u) {
  double theta = x(2);

  // rad/sec
  double wDesired = fabs(u(2));

  // b in meters, direct distance between wheels
  double turnWheelSpeed = (b / 2.0) * wDesired;
  double remainingVelocityBudget = vMax - turnWheelSpeed;

  double vDesired = std::sqrt(pow(u(0), 2) + pow(u(1), 2));

  // only scale vel desired down if it's already above the budget in order to not artificially increase speed
  double budgetCutScalar = (remainingVelocityBudget < vDesired) ? remainingVelocityBudget / vDesired : 1;
  
  u(0) *= budgetCutScalar;
  u(1) *= budgetCutScalar;

  Eigen::Matrix<double, 4, 3> A;
  Eigen::Matrix3d B;
  Eigen::Matrix3d C;

  A << 0.5, 0, 1, 
       0, -0.5, 1, 
       -0.5, 0, 1, 
       0, 0.5, 1;

  B << std::cos(theta), std::sin(theta), 0, 
       -std::sin(theta), std::cos(theta), 0, 
       0, 0, 1;

  C << std::sqrt(2), -std::sqrt(2), 0, 
       std::sqrt(2), std::sqrt(2), 
       0, 0, 0, b;

  Eigen::Vector4d result = A * B * C * u;

  if (velocityCapped) {
    result = result.cwiseMin(-vMax).cwiseMax(vMax);
  }

  return result;
}

Eigen::Vector4d XDrive::linearWheelSpeedToAngular(okapi::QLength wheelDiameter,
                                                  Eigen::Vector4d speeds) {
  speeds /= wheelDiameter.convert(okapi::meter);
  speeds *= 60.0;
  speeds /= PI;

  return speeds;
}

Eigen::Vector4d XDrive::angularWheelSpeedToLinear(okapi::QLength wheelDiameter,
                                          Eigen::Vector4d speeds) {
  speeds *= wheelDiameter.convert(okapi::meter);
  speeds /= 60.0;
  speeds *= PI;

  return speeds;
}

void XDrive::moveLocal(Eigen::Vector3d u) {
  moveGlobal({0, 0, 0}, u);
}

void XDrive::moveGlobal(Eigen::Vector3d x, Eigen::Vector3d u) {
  using namespace okapi::literals;
  auto speeds = ik(x, u);
  speeds = linearWheelSpeedToAngular(3.25_in, speeds);
  // order of motors: NW, NE, SE, SW
  leftFront.moveVelocity(speeds(0));
  rightFront.moveVelocity(speeds(1));
  rightBack.moveVelocity(speeds(2));
  leftBack.moveVelocity(speeds(3));
}

}  // namespace motion