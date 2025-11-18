#pragma once

#include <Arduino.h>
#include <config.h>
#include <math.h>

#define G_MS2 9.80665

struct RotationEuler {
  double x; // roll
  double y; // pitch
  double z; // yaw

  RotationEuler() : x(0.0), y(0.0), z(0.0) {}
  RotationEuler(double rx, double ry, double rz) : x(rx), y(ry), z(rz) {}
};

struct Quaternion {
  double w; // scalar component
  double x; // vector x component
  double y; // vector y component
  double z; // vector z component

  Quaternion() : w(1.0), x(0.0), y(0.0), z(0.0) {}
  Quaternion(double qw, double qx, double qy, double qz)
    : w(qw), x(qx), y(qy), z(qz) {};
};

// m/s^2
struct AccelData {
  double x, y, z;

  AccelData() : x(0.0), y(0.0), z(0.0) {}
  AccelData(double ax, double ay, double az) : x(ax), y(ay), z(az) {}
};

// rad/s
struct GyroData {
  double x, y, z;

  GyroData() : x(0.0), y(0.0), z(0.0) {}
  GyroData(double gx, double gy, double gz) : x(gx), y(gy), z(gz) {}
};

// Conversion helpers
// steps per cm
static const double StepsPerCM =
  (double)STEPS_PER_REV / (WHEEL_DIAMETER_CM * PI);
// cm per step
static const double CMPerStep = 1.0 / StepsPerCM;

#define copysign(x, y) (y < 0.0 ? -fabs(x) : fabs(x))

// Conversions
static inline double cnv_stepsToCM(long steps) {
  return (double)steps * CMPerStep;
}
static inline long cnv_CMToSteps(double cm) { return lround(cm * StepsPerCM); }

static inline void normalizeQuat(Quaternion &q) {
  double norm = sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (norm > 0.0) {
    q.w = q.w / norm;
    q.x = q.x / norm;
    q.y = q.y / norm;
    q.z = q.z / norm;
  }
}
class Rotation {
    public:
  double roll  = 0; // rotation around x-axis
  double pitch = 0; // rotation around y-axis
  double yaw   = 0; // rotation around z-axis

  static Rotation kZero() {
    return Rotation(0.0, 0.0, 0.0);
  }

  double normalizeAngleRads(double angle) {
    while (angle > PI) angle -= TWO_PI;
    while (angle < -PI) angle += TWO_PI;
    return angle;
  }

  double degToRad(double deg) { return deg * PI / 180.0; }
  double radToDeg(double rad) { return rad * 180.0 / PI; }

  // Constructors
  Rotation() : roll(0), pitch(0), yaw(0) {}
  Rotation(double r, double p, double y) { setFromRadians(r, p, y); }

  static Rotation fromDegrees(double r, double p, double y) {
    Rotation rot;
    rot.setFromDegrees(r, p, y);
    return rot;
  }

  Rotation fromRadians(double r, double p, double y) {
    Rotation rot;
    rot.setFromRadians(r, p, y);
    return rot;
  }

  Rotation operator+(const Rotation &r) const {
    return Rotation(roll + r.roll, pitch + r.pitch, yaw + r.yaw);
  }
  Rotation operator-(const Rotation &r) const {
    return Rotation(roll - r.roll, pitch - r.pitch, yaw - r.yaw);
  }

  void setFromRadians(double r, double p, double y) {
    roll  = normalizeAngleRads(r);
    pitch = normalizeAngleRads(p);
    yaw   = normalizeAngleRads(y);
  }

  void setFromRadians(const RotationEuler &euler) {
    setFromRadians(euler.x, euler.y, euler.z);
  }

  // Set rotation from degrees
  void setFromDegrees(double r, double p, double y) {
    roll  = normalizeAngleRads(degToRad(r));
    pitch = normalizeAngleRads(degToRad(p));
    yaw   = normalizeAngleRads(degToRad(y));
  }

  void setFromDegrees(const RotationEuler &euler) {
    setFromDegrees(euler.x, euler.y, euler.z);
  }

  // Set rotation from quaternion
  void setFromQuaternion(Quaternion &q) {
    normalizeQuat(q);

    double w = q.w;
    double x = q.x;
    double y = q.y;
    double z = q.z;

    // Roll (x-axis rotation)
    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    roll             = atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2.0 * (w * y - z * x);
    if (abs(sinp) >= 1.0) {
      pitch = copysign(PI / 2.0, sinp); // use 90 degrees if out of range
    } else {
      pitch = asin(sinp);
    }

    // Yaw (z-axis rotation)
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    yaw              = atan2(siny_cosp, cosy_cosp);
  }

  void setFromQuaternion(double w, double x, double y, double z) {
    Quaternion q = {w, x, y, z};
    setFromQuaternion(q);
  }

  // Get rotation in radians
  RotationEuler getRadians() const {
    RotationEuler result;
    result.x = roll;
    result.y = pitch;
    result.z = yaw;
    return result;
  }

  void getRadians(double &r, double &p, double &y) {
    r = roll;
    p = pitch;
    y = yaw;
  }

  RotationEuler getDegrees() {
    RotationEuler result;
    result.x = radToDeg(roll);
    result.y = radToDeg(pitch);
    result.z = radToDeg(yaw);
    return result;
  }

  void getDegrees(double &r, double &p, double &y) {
    r = radToDeg(roll);
    p = radToDeg(pitch);
    y = radToDeg(yaw);
  }

  Quaternion getQuaternion() {
    // Convert Euler angles to quaternion (ZYX convention)
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
  }

  void getQuaternion(double &w, double &x, double &y, double &z) {
    Quaternion q = getQuaternion();
    w            = q.w;
    x            = q.x;
    y            = q.y;
    z            = q.z;
  }

  double getRollRads() { return roll; }
  double getPitchRads() { return pitch; }
  double getYawRads() { return yaw; }

  double getRollDegs() { return radToDeg(roll); }
  double getPitchDegs() { return radToDeg(pitch); }
  double getYawDegs() { return radToDeg(yaw); }

  void setRollRads(double r) { roll = normalizeAngleRads(r); }
  void setPitchRads(double p) { pitch = normalizeAngleRads(p); }
  void setYawRads(double y) { yaw = normalizeAngleRads(y); }

  void setRollDegs(double r) { roll = normalizeAngleRads(degToRad(r)); }
  void setPitchDegs(double p) { pitch = normalizeAngleRads(degToRad(p)); }
  void setYawDegs(double y) { yaw = normalizeAngleRads(degToRad(y)); }

  void reset() {
    roll  = 0;
    pitch = 0;
    yaw   = 0;
  }

  // Print functions for debugging
  void printRadians() {
    Serial.print("Rotation (rad) - Roll: ");
    Serial.print(roll);
    Serial.print(", Pitch: ");
    Serial.print(pitch);
    Serial.print(", Yaw: ");
    Serial.println(yaw);
  }

  void printDegrees() {
    Serial.print("Rotation (deg) - Roll: ");
    Serial.print(radToDeg(roll));
    Serial.print(", Pitch: ");
    Serial.print(radToDeg(pitch));
    Serial.print(", Yaw: ");
    Serial.println(radToDeg(yaw));
  }

  void printQuaternion() {
    Quaternion q = getQuaternion();
    Serial.print("Quaternion - W: ");
    Serial.print(q.w);
    Serial.print(", X: ");
    Serial.print(q.x);
    Serial.print(", Y: ");
    Serial.print(q.y);
    Serial.print(", Z: ");
    Serial.println(q.z);
  }
};

struct Pose {
  double   x;   // cm
  double   y;   // cm
  Rotation rot; // heading

  Pose() : x(0.0), y(0.0), rot() {}
  Pose(double xcm, double ycm, const Rotation &r) : x(xcm), y(ycm), rot(r) {}

  Pose operator+(const Pose &p) const {
    return Pose(x + p.x, y + p.y, rot + p.rot);
  }
  Pose operator-(const Pose &p) const {
    return Pose(x - p.x, y - p.y, rot - p.rot);
  }
};