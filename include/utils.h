#pragma once

#include <Arduino.h>
#include <math.h>
#include <config.h>

#define G_MS2 9.80665

typedef struct {
  double x; // roll
  double y; // pitch
  double z; // yaw
} RotationEuler;

typedef struct {
  double w; // scalar component
  double x; // vector x component
  double y; // vector y component
  double z; // vector z component
} Quaternion;

// m/s^2
typedef struct {
  double x, y, z;
} AccelData;

// rad/s
typedef struct {
  double x, y, z;
} GyroData;

static const double MMsPerStep = STEPS_PER_REV / (WHEEL_DIAMETER_MM * PI);

#define copysign(x, y) ((x) * (y < 0 ? -1 : 1))

static inline long cnv_stepsToMM(long steps) {
  return steps * MMsPerStep;
}

static inline long cnv_MMToSteps(long mm) {
  return mm / MMsPerStep;
}

static inline void normalizeQuat(Quaternion &q) {
  double norm = sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (norm > 0.0) {
    q.w = q.w / norm;
    q.x = q.x / norm;
    q.y = q.y / norm;
    q.z = q.z / norm;
  }
}

static inline bool clamp(long &value, long min, long max) {
  if (value < min) {
    value = min;
  } else if (value > max) {
    value = max;
  } else {
    return false;
  }

  return true;
}

class Rotation {
private:
  double roll; // rotation around x-axis
  double pitch; // rotation around y-axis
  double yaw; // rotation around z-axis

  double normalizeAngleRads(double angle) {
    while (angle > PI)
      angle -= 2.0 * PI;
    while (angle < -PI)
      angle += 2.0 * PI;
    return angle;
  }

  double degToRad(double deg) { return deg * PI / 180.0; }

  double radToDeg(double rad) const { return rad * 180.0 / PI; }

public:
  // Constructors
  Rotation() : roll(0), pitch(0), yaw(0) {}

  Rotation(double r, double p, double y) : roll(r), pitch(p), yaw(y) {}

  void setFromRadians(double r, double p, double y) {
    roll = normalizeAngleRads(r);
    pitch = normalizeAngleRads(p);
    yaw = normalizeAngleRads(y);
  }

  void setFromRadians(const RotationEuler &euler) {
    setFromRadians(euler.x, euler.y, euler.z);
  }

  // Set rotation from degrees
  void setFromDegrees(double r, double p, double y) {
    roll = normalizeAngleRads(degToRad(r));
    pitch = normalizeAngleRads(degToRad(p));
    yaw = normalizeAngleRads(degToRad(y));
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
    roll = atan2(sinr_cosp, cosr_cosp);

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
    yaw = atan2(siny_cosp, cosy_cosp);
  }

  void setFromQuaternion(double w, double x, double y, double z) {
    Quaternion q = { w, x, y, z };
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

  void getRadians(double &r, double &p, double &y) const {
    r = roll;
    p = pitch;
    y = yaw;
  }

  RotationEuler getDegrees() const {
    RotationEuler result;
    result.x = radToDeg(roll);
    result.y = radToDeg(pitch);
    result.z = radToDeg(yaw);
    return result;
  }

  void getDegrees(double &r, double &p, double &y) const {
    r = radToDeg(roll);
    p = radToDeg(pitch);
    y = radToDeg(yaw);
  }

  Quaternion getQuaternion() const {
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

  void getQuaternion(double &w, double &x, double &y, double &z) const {
    Quaternion q = getQuaternion();
    w = q.w;
    x = q.x;
    y = q.y;
    z = q.z;
  }

  double getRollRads() const { return roll; }

  double getPitchRads() const { return pitch; }

  double getYawRads() const { return yaw; }

  double getRollDegs() const { return radToDeg(roll); }

  double getPitchDegs() const { return radToDeg(pitch); }

  double getYawDegs() const { return radToDeg(yaw); }

  void setRollRads(double r) { roll = normalizeAngleRads(r); }

  void setPitchRads(double p) { pitch = normalizeAngleRads(p); }

  void setYawRads(double y) { yaw = normalizeAngleRads(y); }

  void setRollDegs(double r) { roll = normalizeAngleRads(degToRad(r)); }

  void setPitchDegs(double p) { pitch = normalizeAngleRads(degToRad(p)); }

  void setYawDegs(double y) { yaw = normalizeAngleRads(degToRad(y)); }

  void reset() {
    roll = 0;
    pitch = 0;
    yaw = 0;
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