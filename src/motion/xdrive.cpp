#include "motion/xdrive.hpp"
#include "Eigen/Core"
//#include "Eigen/LU"
#include "Eigen/MatrixFunctions"

namespace motion {
XDrive::XDrive(okapi::Motor &leftFront, okapi::Motor &rightFront,
               okapi::Motor &leftBack, okapi::Motor &rightBack,
               okapi::QSpeed vMax, double b)
    : leftFront(leftFront),
      rightFront(rightFront),
      leftBack(leftBack),
      rightBack(rightBack),
      vMax(vMax.convert(okapi::mps)),
      b(b) {}

double XDrive::calcVelLimit(double theta, double yawRate) {
  double base = fabs(vMax * sqrt(2) / fabs(sin(theta)) + fabs(cos(theta)));
  double scaled = base * fabs((maxYawRate() - yawRate) / maxYawRate());
  return scaled;
}

double XDrive::maxYawRate() {
  auto x = fk(Eigen::Vector3d(0.0, 0.0, 0.0),
              Eigen::Vector4d(vMax, vMax, vMax, vMax));

  return x(2);
}

Eigen::Vector3d XDrive::fk(Eigen::Vector3d x, Eigen::Vector4d u) {
  if (velocityCapped) {
    u = u.cwiseMin(-vMax).cwiseMax(vMax);
  }

  double theta = x(2);

  Eigen::Matrix3d A;
  Eigen::Matrix3d B;
  Eigen::Matrix<double, 3, 4> C; 

  A << cos(theta), -sin(theta), 0, 
       sin(theta), cos(theta), 0, 
       0, 0, 1;
  
  double ootsqt = 1.0 / (2.0 * sqrt(2));

  B << ootsqt, ootsqt, 0,
       -ootsqt, ootsqt, 0,
       0, 0, 1.0 / b;
  
  C << 1, 0, -1, 0,
       0, -1, 0, 1,
       0.25, 0.25, 0.25, 0.25;

  return A * B * C * u;
}

Eigen::Vector4d XDrive::ik(Eigen::Vector3d x, Eigen::Vector3d u) {
  double theta = x(2);

  Eigen::Matrix<double, 4, 3> A;
  Eigen::Matrix3d B;
  Eigen::Matrix3d C;

  A << 0.5, 0, 1,
       0, -0.5, 1,
       -0.5, 0, 1,
       0, 0.5, 1;
  
  B << cos(theta), sin(theta), 0,
       -sin(theta), cos(theta), 0,
       0, 0, 1;
  
  C << sqrt(2), -sqrt(2), 0,
       sqrt(2), sqrt(2), 0,
       0, 0, b;
  
  Eigen::Vector4d result = A * B * C * u;

  if (velocityCapped) {
    result = result.cwiseMin(-vMax).cwiseMax(vMax);
  }

  return result;
}

}  // namespace motion