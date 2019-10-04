#pragma once

#include "okapi/api/chassis/model/chassisModel.hpp"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QLength.hpp"

namespace structs {
using namespace okapi::literals;

// A class for a 2 dimensional vector, using okapi units, designed for position
class PositionVector {
  okapi::QLength x;
  okapi::QLength y;

 public:
  PositionVector();
  PositionVector(okapi::QLength x, okapi::QLength y);
  PositionVector(okapi::QAngle theta);

  static PositionVector add(PositionVector a, PositionVector b);
  static PositionVector subtract(PositionVector a, PositionVector b);
  static PositionVector multiply(PositionVector a, double n);
  static PositionVector divide(PositionVector a, double n);
  static PositionVector limit(PositionVector a, okapi::QLength limit);
  static PositionVector rotate(PositionVector a, okapi::QAngle theta);
  static PositionVector normalize(PositionVector a,
                                  okapi::QLength unit = okapi::meter);
  static okapi::QLength distanceBetween(PositionVector a, PositionVector b);
  static okapi::QAngle angleBetween(PositionVector a, PositionVector b);
  static okapi::QLength dot(PositionVector a, PositionVector b);

  void setSelf(PositionVector other);
  void addSelf(PositionVector other);
  void subtractSelf(PositionVector other);
  void multiplySelf(double n);
  void divideSelf(double n);
  void limitSelf(okapi::QLength n);
  void rotateSelf(okapi::QAngle theta);
  void normalizeSelf(okapi::QLength unit = okapi::meter);

  // TODO make a bunch of statics, then layer these ontop
  okapi::QLength distanceTo(PositionVector other);
  okapi::QAngle angleTo(PositionVector other);
  okapi::QLength dot(PositionVector other);

  okapi::QLength getX();
  okapi::QLength getY();
  okapi::QAngle getTheta();
  okapi::QLength getMag();
  okapi::QLength getMagSquared();
};

// TODO make this a class and encapsulate me daddy
struct Pose {
  okapi::QAngle heading;
  PositionVector position;

  Pose();
  Pose(okapi::QLength x, okapi::QLength y, okapi::QAngle heading = 0_deg);
  Pose(PositionVector position, okapi::QAngle heading = 0_deg);

  void turn(okapi::QAngle headingChange);  // TODO constrain this reee
};

struct DriveVectorCommand {
  double forward;
  double yaw;

  DriveVectorCommand(double forward, double yaw);

  void execute(okapi::ChassisModel& model);
};

struct KinematicConstraints {
  double maxVel;
  double maxAccel;
  double maxJerk;

 public:
  KinematicConstraints(double maxVel, double maxAccel, double maxJerk = 0.0) {};
};

struct PIDGains {
  double kP;
  double kI;
  double kD;

 public:
  PIDGains(double kP, double kI, double kD) : kP(kP), kI(kI), kD(kD) {};
};
}  // namespace mlib