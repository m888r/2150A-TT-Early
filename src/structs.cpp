#include "structs.hpp"

namespace structs {
using namespace okapi;
using namespace okapi::literals;

PositionVector::PositionVector(QLength x, QLength y)
    : x(x)
    , y(y) {}
PositionVector::PositionVector()
    : PositionVector(0_m, 0_m) {}
PositionVector::PositionVector(QAngle theta) {}

PositionVector PositionVector::add(PositionVector a, PositionVector b) {
    return PositionVector(a.x + b.x, a.y + b.y);
}

PositionVector PositionVector::subtract(PositionVector a, PositionVector b) {
    return PositionVector(a.x - b.x, a.y - b.y);
}

PositionVector PositionVector::multiply(PositionVector a, double n) {
    return PositionVector(a.x * n, a.y * n);
}

PositionVector PositionVector::divide(PositionVector a, double n) {
    return multiply(a, 1.0 / n);
}

PositionVector PositionVector::limit(PositionVector a, QLength n) {
    if (a.getMagSquared().convert(meter) > pow(n.convert(meter), 2)) {
        return multiply(
            normalize(a, meter),
            n.convert(meter));
    }

    return a;
}

PositionVector PositionVector::rotate(PositionVector a, okapi::QAngle theta) {
    return PositionVector(
        a.x * std::cos(theta.convert(radian)) - a.y * std::sin(theta.convert(radian)),
        a.x * std::sin(theta.convert(radian)) + a.y * std::cos(theta.convert(radian)));
}

PositionVector PositionVector::normalize(PositionVector a, okapi::QLength unit) {
    return divide(
        a,
        a.getMag().convert(unit));
}

okapi::QLength PositionVector::distanceBetween(PositionVector a, PositionVector b) {
    auto dX = (a.x - b.x).convert(meter);
    auto dY = (a.y - b.y).convert(meter);

    return std::sqrt(dX * dX + dY * dY) * meter;
}

okapi::QAngle PositionVector::angleBetween(PositionVector a, PositionVector b) {
    return subtract(a, b).getTheta();
}

okapi::QLength PositionVector::dot(PositionVector a, PositionVector b) {
    return meter * (a.x.convert(meter) * b.x.convert(meter) + a.y.convert(meter) * b.y.convert(meter));
}

void PositionVector::setSelf(PositionVector other) {
    x = other.x;
    y = other.y;
}

void PositionVector::addSelf(PositionVector other) {
    setSelf(add(*this, other));
}

void PositionVector::subtractSelf(PositionVector other) {
    setSelf(subtract(*this, other));
}

void PositionVector::multiplySelf(double n) {
    setSelf(multiply(*this, n));
}

void PositionVector::divideSelf(double n) {
    multiplySelf(1.0 / n);
}

void PositionVector::limitSelf(QLength n) {
    setSelf(limit(*this, n));
}

void PositionVector::rotateSelf(QAngle theta) {
    setSelf(rotate(*this, theta));
}

void PositionVector::normalizeSelf(QLength unit) {
    setSelf(normalize(*this, unit));
}

QLength PositionVector::distanceTo(PositionVector other) {
    return distanceBetween(*this, other);
}

QAngle PositionVector::angleTo(PositionVector other) {
    return angleBetween(*this, other);
}

QLength PositionVector::dot(PositionVector other) {
    return dot(*this, other);
}

QLength PositionVector::getX() {
    return x;
}

QLength PositionVector::getY() {
    return y;
}

QAngle PositionVector::getTheta() {
    return radian * std::atan2(x.convert(meter), y.convert(meter));
}

QLength PositionVector::getMag() {
    return meter * std::sqrt(pow(x.convert(meter), 2) + pow(y.convert(meter), 2));
}

QLength PositionVector::getMagSquared() {
    return meter * (pow(x.convert(meter), 2) + pow(y.convert(meter), 2));
}

Pose::Pose(okapi::QLength x, okapi::QLength y, okapi::QAngle heading)
    : position(x, y)
    , heading(heading) {}
Pose::Pose()
    : Pose::Pose(0_m, 0_m, 0_deg) {}

Pose::Pose(PositionVector position, okapi::QAngle heading)
    : position(position)
    , heading(heading) {}

void Pose::turn(QAngle headingChange) {
    heading += headingChange;
}

DriveVectorCommand::DriveVectorCommand(double forward, double yaw)
    : forward(forward)
    , yaw(yaw) {}

void DriveVectorCommand::execute(ChassisModel& model) {
    model.driveVector(forward, yaw);
}

KinematicConstraints::KinematicConstraints(double maxVel, double maxAccel, double maxJerk)
    : maxVel(maxVel)
    , maxAccel(maxAccel)
    , maxJerk(maxJerk) {}
}