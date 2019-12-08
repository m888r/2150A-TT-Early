#include "okapi/api.hpp"
#include "Eigen/Core"
#include "Eigen/LU"
#include "Eigen/MatrixFunctions"

namespace motion {
class XDrive {
  XDrive(okapi::Motor &leftFront, okapi::Motor &rightFront,
         okapi::Motor &leftBack, okapi::Motor &rightBack, okapi::QSpeed vMax,
         double b);
  
  // state vector (x, y, theta), control vector (v1, v2, v3, v4)
  Eigen::Matrix<double, 4, 4> fk(Eigen::Matrix<double, 3, 1> x, Eigen::Matrix<double, 4, 1> u) {

  }
};
}  // namespace motion
