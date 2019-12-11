#include "Eigen/Core"
//#include "Eigen/LU"
#include "Eigen/MatrixFunctions"
#include "okapi/api.hpp"

namespace motion {
class XDrive {
 public:
  XDrive(okapi::Motor &leftFront, okapi::Motor &rightFront,
         okapi::Motor &leftBack, okapi::Motor &rightBack, okapi::QSpeed vMax,
         double b);

  /**
   * @param x state vector (x, y, theta),
   * @param u control vector (v1, v2, v3, v4)
   * @return (ẋ, ẏ, w)
   */ 
  Eigen::Vector3d fk(Eigen::Vector3d x, Eigen::Vector4d u);

  /**
   * @param x state vector (x, y, theta)
   * @param u control vector (ẋ, ẏ, w)
   * @return (v1, v2, v3, v4)
   */
  Eigen::Vector4d ik(Eigen::Vector3d x, Eigen::Vector3d u);

 private:
  okapi::Motor leftFront;
  okapi::Motor rightFront;
  okapi::Motor leftBack;
  okapi::Motor rightBack;
  double vMax;
  double b;
  bool velocityCapped = false;

  /**
   * @param theta current angle in radians
   */
  double calcVelLimit(double theta, double yawRate);

  double maxYawRate();
};
}  // namespace motion
