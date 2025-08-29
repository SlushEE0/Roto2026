#include <math.h>
#include "kalman.h"
#include "utils.h"

// ---- Kalman filter state (1D yaw fusion in radians) ----

// Estimated yaw (radians, 0–2π)
static double x_k = 0.0;

// Error covariance (uncertainty of estimate)
static double P_k = 1.0;

// Process noise covariance (trust in wheels)
static const double Q = 0.001; // tweak this

// Measurement noise covariance (trust in IMU)
static const double R = 0.05; // tweak this

static RotationEuler currRotation = { 0, 0, 0 };

Kalman::Kalman() {
  x_k = 0.0;
  P_k = 1.0;
}

// ---- Prediction step (using wheel yaw) ----
void Kalman::updateWheels(double *yaw) {
  // Prediction: trust wheel angle as input
  x_k = *yaw;

  // Increase uncertainty slightly
  P_k = P_k + Q;

  // Wrap to [0, 2π)
  x_k = fmod(x_k, 2.0 * PI);
  if (x_k < 0)
    x_k += 2.0 * PI;

  currRotation.z = x_k;
  currRotation.x = 0.0;
  currRotation.y = 0.0;
}

// ---- Correction step (using IMU yaw) ----
void Kalman::updateIMU(RotationEuler *rotation) {
  RotationEuler euler = *rotation;
  double z = euler.z; // IMU yaw (radians)

  // Innovation (residual between IMU and prediction)
  double y = z - x_k;

  // Normalize residual to [-π, π)
  if (y > PI)
    y -= 2.0 * PI;
  if (y < -PI)
    y += 2.0 * PI;

  // Kalman gain
  double K = P_k / (P_k + R);

  // Update estimate
  x_k = x_k + K * y;

  // Update uncertainty
  P_k = (1 - K) * P_k;

  // Wrap angle again
  x_k = fmod(x_k, 2.0 * PI);
  if (x_k < 0)
    x_k += 2.0 * PI;

  currRotation.z = x_k;
  currRotation.x = 0.0;
  currRotation.y = 0.0;
}


