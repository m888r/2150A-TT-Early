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
  Eigen::Matrix4d fk(Eigen::Vector3d x, Eigen::Vector4d u);
};
}  // namespace motion
