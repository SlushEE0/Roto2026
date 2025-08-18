#include <BasicLinearAlgebra.h>
#include <math.h>
#include "utils.h"

#include "kalman.h"

using namespace BLA;

static inline Matrix<4, 4, double> eye4(double s = 1.0);

static Matrix<4, 1, double> x_k = { 1, 0, 0, 0 };
static Matrix<4, 4, double> P_k = eye4();

static Matrix<4, 1, double> x_p = { 1, 0, 0, 0 };
static Matrix<4, 4, double> P_p = eye4() * 2.0;

static Matrix<4, 4, double> H = eye4();
static Matrix<4, 4, double> Q = eye4() * 0.0005;
static Matrix<4, 4, double> R = eye4() * 20.0;

static unsigned long prevUpdateUS = 0;
static RotationEuler currRotation = { 0, 0, 0 };

Kalman::Kalman() {
  prevUpdateUS = micros();
}

static inline Matrix<4, 4, double> eye4(double s) {
  Matrix<4, 4, double> I;
  I.Fill(0);
  I(0, 0) = s;
  I(1, 1) = s;
  I(2, 2) = s;
  I(3, 3) = s;
  return I;
}

static inline Matrix<4, 4, double> getA(GyroData *gyro, long dt_us) {
  double dt_s = dt_us * 1E-06;

  double w1 = gyro->x;
  double w2 = gyro->y;
  double w3 = gyro->z;

  auto i_4 = eye4();
  Matrix<4, 4, double> m = { 0,  -w1, -w2, -w3, w1, 0,  w3,  -w2,
                             w2, -w3, 0,   w1,  w3, w2, -w1, 0 };

  return i_4 + dt_s * 0.5 * m;
}

static inline Matrix<4, 1, double> getEulerParams(GyroData *gyro) {
  double euler[3] = { gyro->z, gyro->x, gyro->y };

  Matrix<4, 1, double> q;
  RotationQuat quat;

  double c1 = cos(euler[0] * 0.5);
  double s1 = sin(euler[0] * 0.5);
  double c2 = cos(euler[1] * 0.5);
  double s2 = sin(euler[1] * 0.5);
  double c3 = cos(euler[2] * 0.5);
  double s3 = sin(euler[2] * 0.5);

  quat.w = c1 * c2 * c3 + s1 * s2 * s3;
  quat.i = c1 * c2 * s3 - s1 * s2 * c3;
  quat.j = c1 * s2 * c3 + s1 * c2 * s3;
  quat.k = s1 * c2 * c3 - c1 * s2 * s3;

  normalizeQuat(quat);

  q(0) = quat.w;
  q(1) = quat.i;
  q(2) = quat.j;
  q(3) = quat.k;

  return q;
}

static inline RotationEuler eulerParamsTo321(Matrix<4, 1, double> ep) {
  RotationEuler rotation = { 0, 0, 0 };
  RotationQuat quat = { ep(0), ep(1), ep(2), ep(3) };

  normalizeQuat(quat);

  double q0 = quat.w;
  double q1 = quat.i;
  double q2 = quat.j;
  double q3 = quat.k;

  double z =
    atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
  double y = asin(-2 * (q1 * q3 - q0 * q2));
  double x =
    atan2(2 * (q2 * q3 + q0 * q1), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);

  rotation.x = fmod((x * 180 / PI) + 720.0, 360.0);
  rotation.y = fmod((y * 180 / PI) + 720.0, 360.0);
  rotation.z = fmod((z * 180 / PI) + 720.0, 360.0);

  return rotation;
}

static inline void kPredict(GyroData *gyroData, long dt_us) {
  auto A = getA(gyroData, dt_us);

  x_p = A * x_k;
  P_p = A * P_k * ~A + Q;
}

static inline void kEstimate(const Matrix<4, 1, double> &z) {
  // Kalman Gain
  auto K = P_p * ~H * Inverse(H * P_p * ~H + R);

  // Update
  x_k = x_p + K * (z - H * x_p);
  P_k = P_p - K * H * P_p;
}

void Kalman::update(GyroData *gyroData, AccelData * accelData) {
  unsigned long now_us = micros();
  long dt_us = now_us - prevUpdateUS;
  if (dt_us <= 0 || dt_us > 500000) {
    dt_us = 16000; // fallback
  }
  prevUpdateUS = now_us;

  kPredict(gyroData, dt_us);

  auto z = getEulerParams(gyroData);
  kEstimate(z);
  
  currRotation = eulerParamsTo321(x_k);
}

RotationEuler Kalman::getRotationEuler() {
  return currRotation;
}

RotationQuat Kalman::getRotationQuat() {
  return cnv_eulerToQuat(&currRotation);
}