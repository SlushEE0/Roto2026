#pragma once

#include <Arduino.h>
#include <config.h>
#include <math.h>

// Use float for STM32F103 (no hardware FPU - double is emulated and slow)
#define G_MS2 9.80665f

// Float constants
#define PI_F 3.14159265f
#define TWO_PI_F 6.28318530f

struct RotationEuler {
  float x; // roll
  float y; // pitch
  float z; // yaw

  RotationEuler() : x(0.0f), y(0.0f), z(0.0f) {}
  RotationEuler(float rx, float ry, float rz) : x(rx), y(ry), z(rz) {}
};

struct Quaternion {
  float w; // scalar component
  float x; // vector x component
  float y; // vector y component
  float z; // vector z component

  Quaternion() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}
  Quaternion(float qw, float qx, float qy, float qz)
    : w(qw), x(qx), y(qy), z(qz) {};
};

// m/s^2
struct AccelData {
  float x, y, z;

  AccelData() : x(0.0f), y(0.0f), z(0.0f) {}
  AccelData(float ax, float ay, float az) : x(ax), y(ay), z(az) {}
};

// rad/s
struct GyroData {
  float x, y, z;

  GyroData() : x(0.0f), y(0.0f), z(0.0f) {}
  GyroData(float gx, float gy, float gz) : x(gx), y(gy), z(gz) {}
};

// Conversion helpers
// steps per cm
static const float StepsPerCM =
  (float)STEPS_PER_REV / (WHEEL_DIAMETER_CM * PI_F);
// cm per step
static const float CMPerStep = 1.0f / StepsPerCM;

#define copysignf_local(x, y) ((y) < 0.0f ? -fabsf(x) : fabsf(x))

// Conversions
static inline float cnv_stepsToCM(long steps) {
  return (float)steps * CMPerStep;
}
static inline long cnv_CMToSteps(float cm) { return lroundf(cm * StepsPerCM); }

static inline float clampFloat(float val, float minVal, float maxVal) {
  if (val < minVal)
    return minVal;
  if (val > maxVal)
    return maxVal;
  return val;
}

static inline void normalizeQuat(Quaternion &q) {
  float norm = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (norm > 0.0f) {
    q.w = q.w / norm;
    q.x = q.x / norm;
    q.y = q.y / norm;
    q.z = q.z / norm;
  }
}
class Rotation {
    public:
  float roll  = 0; // rotation around x-axis
  float pitch = 0; // rotation around y-axis
  float yaw   = 0; // rotation around z-axis

  static Rotation kZero() {
    return Rotation(0.0f, 0.0f, 0.0f);
  }

  float normalizeAngleRads(float angle) {
    while (angle > PI_F) angle -= TWO_PI_F;
    while (angle < -PI_F) angle += TWO_PI_F;
    return angle;
  }

  float degToRad(float deg) { return deg * PI_F / 180.0f; }
  float radToDeg(float rad) { return rad * 180.0f / PI_F; }

  // Constructors
  Rotation() : roll(0), pitch(0), yaw(0) {}
  Rotation(float r, float p, float y) { setFromRadians(r, p, y); }

  static Rotation fromDegrees(float r, float p, float y) {
    Rotation rot;
    rot.setFromDegrees(r, p, y);
    return rot;
  }

  Rotation fromRadians(float r, float p, float y) {
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

  void setFromRadians(float r, float p, float y) {
    roll  = normalizeAngleRads(r);
    pitch = normalizeAngleRads(p);
    yaw   = normalizeAngleRads(y);
  }

  void setFromRadians(const RotationEuler &euler) {
    setFromRadians(euler.x, euler.y, euler.z);
  }

  // Set rotation from degrees
  void setFromDegrees(float r, float p, float y) {
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

    float w = q.w;
    float x = q.x;
    float y = q.y;
    float z = q.z;

    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (w * x + y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    roll             = atan2f(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (w * y - z * x);
    if (fabsf(sinp) >= 1.0f) {
      pitch = copysignf_local(PI_F / 2.0f, sinp); // use 90 degrees if out of range
    } else {
      pitch = asinf(sinp);
    }

    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    yaw              = atan2f(siny_cosp, cosy_cosp);
  }

  void setFromQuaternion(float w, float x, float y, float z) {
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

  void getRadians(float &r, float &p, float &y) {
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

  void getDegrees(float &r, float &p, float &y) {
    r = radToDeg(roll);
    p = radToDeg(pitch);
    y = radToDeg(yaw);
  }

  Quaternion getQuaternion() {
    // Convert Euler angles to quaternion (ZYX convention)
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
  }

  void getQuaternion(float &w, float &x, float &y, float &z) {
    Quaternion q = getQuaternion();
    w            = q.w;
    x            = q.x;
    y            = q.y;
    z            = q.z;
  }

  float getRollRads() const { return roll; }
  float getPitchRads() const { return pitch; }
  float getYawRads() const { return yaw; }

  float getRollDegs() const { return roll * 180.0f / PI_F; }
  float getPitchDegs() const { return pitch * 180.0f / PI_F; }
  float getYawDegs() const { return yaw * 180.0f / PI_F; }

  void setRollRads(float r) { roll = normalizeAngleRads(r); }
  void setPitchRads(float p) { pitch = normalizeAngleRads(p); }
  void setYawRads(float y) { yaw = normalizeAngleRads(y); }

  void setRollDegs(float r) { roll = normalizeAngleRads(degToRad(r)); }
  void setPitchDegs(float p) { pitch = normalizeAngleRads(degToRad(p)); }
  void setYawDegs(float y) { yaw = normalizeAngleRads(degToRad(y)); }

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
  float    x;   // cm
  float    y;   // cm
  Rotation rot; // heading

  Pose() : x(0.0f), y(0.0f), rot() {}
  Pose(float xcm, float ycm, const Rotation &r) : x(xcm), y(ycm), rot(r) {}

  Pose operator+(const Pose &p) const {
    return Pose(x + p.x, y + p.y, rot + p.rot);
  }
  Pose operator-(const Pose &p) const {
    return Pose(x - p.x, y - p.y, rot - p.rot);
  }
};