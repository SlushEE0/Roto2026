#include <BasicLinearAlgebra.h>
#include <math.h>
#include "utils.h"

#include "kalman.h"

using namespace BLA;

//===========================
// Helpers / Identities
//===========================
static inline Matrix<3, 3, double> eye3(double s = 1.0) {
  Matrix<3, 3, double> I;
  I.Fill(0);
  I(0, 0) = s;
  I(1, 1) = s;
  I(2, 2) = s;
  return I;
}

static inline Matrix<4, 4, double> eye4(double s = 1.0) {
  Matrix<4, 4, double> I;
  I.Fill(0);
  I(0, 0) = s;
  I(1, 1) = s;
  I(2, 2) = s;
  I(3, 3) = s;
  return I;
}

static inline Matrix<6, 6, double> eye6(double s = 1.0) {
  Matrix<6, 6, double> I;
  I.Fill(0);
  for (int i = 0; i < 6; ++i)
    I(i, i) = s;
  return I;
}

static inline Matrix<3, 3, double> skew3(double x, double y, double z) {
  Matrix<3, 3, double> S = { 0, -z, y, z, 0, -x, -y, x, 0 };
  return S;
}

//===========================
// Tuning (BNO085-oriented)
//===========================
// Gyro noise density (rad/s/√Hz)
static constexpr double GYRO_NOISE_DENSITY = 0.0001;
// Gyro bias random-walk density (rad/s/√Hz)
static constexpr double GYRO_BIAS_RW_DENSITY = 1e-4;
// Accel unit-vector noise (std dev) for gravity direction (unitless)
static constexpr double ACCEL_UNIT_STD_GOOD = 0.5; // calm
static constexpr double ACCEL_UNIT_STD_OK = 1.0; // some linear accel
// Accel magnitude gating bands
static constexpr double ACC_MAG_GOOD_LOW = 0.93;
static constexpr double ACC_MAG_GOOD_HIGH = 1.07;
static constexpr double ACC_MAG_OK_LOW = 0.85;
static constexpr double ACC_MAG_OK_HIGH = 1.15;

//===========================
// Filter State (MEKF: error-state + bias)
//===========================
// Nominal attitude as quaternion [w,x,y,z]^T
static Matrix<4, 1, double> q_hat = { 1, 0, 0, 0 };
// Error-state x = [dtheta(3); bias(3)]
static Matrix<6, 1, double> x_err = { 0, 0, 0, 0, 0, 0 };
// Covariance
static Matrix<6, 6, double> P = eye6() * 10.0; // modest initial uncertainty

static unsigned long prevUpdateUS = 0;
static RotationEuler currRotation = { 0, 0, 0 };

//===========================
// Quaternion utilities
//===========================
static inline void normalizeQuatVec(Matrix<4, 1, double> &qv) {
  RotationQuat q = { qv(0), qv(1), qv(2), qv(3) };
  normalizeQuat(q);
  qv(0) = q.w;
  qv(1) = q.i;
  qv(2) = q.j;
  qv(3) = q.k;
}

static inline void quatMultiply(const Matrix<4, 1, double> &qa,
                                const Matrix<4, 1, double> &qb,
                                Matrix<4, 1, double> &qout) {
  double wa = qa(0), xa = qa(1), ya = qa(2), za = qa(3);
  double wb = qb(0), xb = qb(1), yb = qb(2), zb = qb(3);
  qout(0) = wa * wb - xa * xb - ya * yb - za * zb;
  qout(1) = wa * xb + xa * wb + ya * zb - za * yb;
  qout(2) = wa * yb - xa * zb + ya * wb + za * xb;
  qout(3) = wa * zb + xa * yb - ya * xb + za * wb;
}

static inline void applySmallAngleToQuat(Matrix<4, 1, double> &q,
                                         const Matrix<3, 1, double> &dtheta) {
  Matrix<4, 1, double> dq = { 1.0,
                              0.5 * dtheta(0),
                              0.5 * dtheta(1),
                              0.5 * dtheta(2) };
  Matrix<4, 1, double> qnew;
  quatMultiply(dq, q, qnew); // left-multiplicative correction
  q = qnew;
  normalizeQuatVec(q);
}

// Integrate quaternion with body rates (rad/s) over dt (s)
static inline void integrateQuat(Matrix<4, 1, double> &q,
                                 const double wx,
                                 const double wy,
                                 const double wz,
                                 const double dt) {
  // Small-angle integration: q <- (I + 0.5*dt*Omega(omega)) q
  Matrix<4, 4, double> Omega = { 0,  -wx, -wy, -wz, wx, 0,  wz,  -wy,
                                 wy, -wz, 0,   wx,  wz, wy, -wx, 0 };
  auto A = eye4() + 0.5 * dt * Omega;
  q = A * q;
  normalizeQuatVec(q);
}

// Predicted body-frame gravity direction from quaternion (unit vector)
static inline Matrix<3, 1, double>
gravityBodyFromQuat(const Matrix<4, 1, double> &q) {
  double w = q(0), x = q(1), y = q(2), z = q(3);
  // Row 3 of R(body->world): world Z expressed in body coordinates
  Matrix<3, 1, double> gb = { 2.0 * (x * z - w * y),
                              2.0 * (y * z + w * x),
                              1.0 - 2.0 * (x * x + y * y) };
  return gb; // near-unit if q is normalized
}

Kalman::Kalman() {
  prevUpdateUS = micros();
}

//===========================
// Predict
//===========================
static inline void kPredict(const GyroData *gyroData, long dt_us) {
  double dt = (dt_us > 0) ? (dt_us * 1e-6) : 0.0;

  // Unpack bias from error-state
  Matrix<3, 1, double> b = { x_err(3), x_err(4), x_err(5) };

  // Gyro measurement (rad/s) minus bias estimate
  double wx = gyroData->x - b(0);
  double wy = gyroData->y - b(1);
  double wz = gyroData->z - b(2);

  // Propagate nominal quaternion
  integrateQuat(q_hat, wx, wy, wz, dt);

  // Build continuous-time F for error-state: [ -[w]_x  -I; 0 0 ]
  auto Sw = skew3(wx, wy, wz);
  Matrix<6, 6, double> Fm;
  Fm.Fill(0);
  // Top-left 3x3
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      Fm(r, c) = -Sw(r, c);
  // Top-right 3x3 = -I
  for (int i = 0; i < 3; ++i)
    Fm(i, 3 + i) = -1.0;

  // Discrete transition Phi ≈ I + F*dt
  auto Phi = eye6() + Fm * dt;

  // Discrete process noise Qd (first-order): diag(ng^2*dt*I3, nb^2*dt*I3)
  double qg = (GYRO_NOISE_DENSITY * GYRO_NOISE_DENSITY) * dt; // rad^2
  double qb = (GYRO_BIAS_RW_DENSITY * GYRO_BIAS_RW_DENSITY) * dt; // (rad/s)^2
  Matrix<6, 6, double> Qd;
  Qd.Fill(0);
  for (int i = 0; i < 3; ++i) {
    Qd(i, i) = qg;
    Qd(3 + i, 3 + i) = qb;
  }

  // Propagate covariance
  P = Phi * P * ~Phi + Qd;
}

//===========================
// Update (accelerometer gravity direction)
//===========================
static inline void kUpdateAccel(const AccelData *accelData) {
  // Gate on accel magnitude
  double ax = accelData->x, ay = accelData->y, az = accelData->z;
  double an = sqrt(ax * ax + ay * ay + az * az);
  if (an <= 1e-6)
    return; // no usable measurement

  double ratio = an / G_MS2;
  double sigma;
  bool use = false;
  if (ratio > ACC_MAG_GOOD_LOW && ratio < ACC_MAG_GOOD_HIGH) {
    sigma = ACCEL_UNIT_STD_GOOD;
    use = true;
  } else if (ratio > ACC_MAG_OK_LOW && ratio < ACC_MAG_OK_HIGH) {
    sigma = ACCEL_UNIT_STD_OK;
    use = true;
  } else {
    use = false; // heavy linear acceleration: skip
  }
  if (!use)
    return;

  // Form unit accel (body-frame gravity direction measurement)
  Matrix<3, 1, double> z = { ax / an, ay / an, az / an };

  // Predicted body gravity from attitude
  Matrix<3, 1, double> h = gravityBodyFromQuat(q_hat);

  // Residual r = z - h
  Matrix<3, 1, double> r = z - h;

  // Measurement jacobian H = [ -[h]_x  0 ]
  auto Sh = skew3(h(0), h(1), h(2));
  Matrix<3, 6, double> Hm; // 3x6
  for (int r0 = 0; r0 < 3; ++r0) {
    for (int c0 = 0; c0 < 3; ++c0) {
      Hm(r0, c0) = -Sh(r0, c0);
      Hm(r0, 3 + c0) = 0.0;
    }
  }

  // R (3x3)
  auto Rm = eye3(sigma * sigma);

  // Kalman gain K = P H^T (H P H^T + R)^-1
  auto S = Hm * P * ~Hm + Rm;
  auto K = P * ~Hm * Inverse(S); // 6x3

  // State update
  auto dx = K * r; // 6x1
  for (int i = 0; i < 6; ++i)
    x_err(i) += dx(i);

  // Joseph-form covariance update
  auto I6 = eye6();
  auto I_KH = I6 - K * Hm;
  P = I_KH * P * ~I_KH + K * Rm * ~K;

  // Apply attitude correction and reset attitude error
  Matrix<3, 1, double> dth = { x_err(0), x_err(1), x_err(2) };
  applySmallAngleToQuat(q_hat, dth);

  // Reset attitude error to zero
  x_err(0) = 0.0;
  x_err(1) = 0.0;
  x_err(2) = 0.0;

  // Covariance reset (first-order): top-left block premultiplied by (I -
  // 0.5*[dth]_x)
  auto Sd = skew3(dth(0), dth(1), dth(2));
  Matrix<6, 6, double> T = eye6();
  for (int r0 = 0; r0 < 3; ++r0)
    for (int c0 = 0; c0 < 3; ++c0)
      T(r0, c0) = T(r0, c0) - 0.5 * Sd(r0, c0);
  P = T * P * ~T;
}

void Kalman::update(GyroData *gyroData, AccelData *accelData) {
  unsigned long now_us = micros();
  long dt_us = now_us - prevUpdateUS;
  if (dt_us <= 0 || dt_us > 500000)
    dt_us = 10000; // fallback
  prevUpdateUS = now_us;

  // Predict with current gyro (subtracting estimated bias)
  kPredict(gyroData, dt_us);

  // Update with accel gravity direction (if valid)
  kUpdateAccel(accelData);

  // Prepare outputs
  RotationQuat qout = { q_hat(0), q_hat(1), q_hat(2), q_hat(3) };
  currRotation = cnv_quatToEuler(&qout);
}

RotationEuler Kalman::getRotationEuler() {
  return currRotation;
}

RotationQuat Kalman::getRotationQuat() {
  RotationQuat qout = { q_hat(0), q_hat(1), q_hat(2), q_hat(3) };
  return qout;
}