#include "odometry.h"

#include <math.h>

// stm32f103 only has hardware float compute
#define sinf_local(x) sinf(x)
#define cosf_local(x) cosf(x)

namespace {
template <int R, int C>
void zeroMatrix(BLA::Matrix<R, C, float>& m) {
  for (int r = 0; r < R; ++r) {
    for (int c = 0; c < C; ++c) {
      m(r, c) = 0.0f;
    }
  }
}

template <int N>
BLA::Matrix<N, N, float> identityMatrix() {
  BLA::Matrix<N, N, float> mat;
  zeroMatrix(mat);
  for (int i = 0; i < N; ++i) {
    mat(i, i) = 1.0f;
  }
  return mat;
}

} // namespace

Odometry::Odometry() {
  zeroMatrix(m_state);
  zeroMatrix(m_P);
  m_slipDetected = false;
  reset();
}

void Odometry::reset(const Pose& pose) {
  m_state(0, 0) = pose.x;
  m_state(1, 0) = pose.y;
  m_state(2, 0) = pose.rot.getYawRads();

  zeroMatrix(m_P);
  m_P(0, 0) = 1.0f;   // 1 cm^2 variance in x
  m_P(1, 1) = 1.0f;   // 1 cm^2 variance in y
  m_P(2, 2) = 0.01f;  // ~0.1 rad^2 variance in theta (~6 degrees)
  
  m_slipDetected = false;
}

void Odometry::setNoise(const OdometryNoiseConfig& cfg) {
  m_noise = cfg;
}

void Odometry::predict(float v, float omega, float dt) {
  if (dt < 1e-6f) {
    return;
  }

  // Current state
  float x     = m_state(0, 0);
  float y     = m_state(1, 0);
  float theta = m_state(2, 0);

  float cosTheta = cosf_local(theta);
  float sinTheta = sinf_local(theta);

  // Predict new state
  float xNew     = x + v * cosTheta * dt;
  float yNew     = y + v * sinTheta * dt;
  float thetaNew = normalizeAngle(theta + omega * dt);

  // Update state
  m_state(0, 0) = xNew;
  m_state(1, 0) = yNew;
  m_state(2, 0) = thetaNew;

  // jacobian
  //
  // F = | 1  0  -v*sin(theta)*dt |
  //     | 0  1   v*cos(theta)*dt |
  //     | 0  0   1               |

  BLA::Matrix<3, 3, float> F_jac = identityMatrix<3>();
  F_jac(0, 2) = -v * sinTheta * dt;
  F_jac(1, 2) =  v * cosTheta * dt;

  BLA::Matrix<3, 3, float> Q;
  zeroMatrix(Q);

  float vMag = fabsf(v);
  float omegaMag = fabsf(omega);

  Q(0, 0) = m_noise.qX * dt + 0.001f * vMag * dt;
  Q(1, 1) = m_noise.qY * dt + 0.001f * vMag * dt;

  Q(2, 2) = m_noise.qTheta * dt + 0.01f * omegaMag * dt;

  BLA::Matrix<3, 3, float> Ft = ~F_jac;  // Transpose
  m_P = F_jac * m_P * Ft + Q;

  enforceSymmetry();
}

void Odometry::correct(float imuYaw) {
  BLA::Matrix<1, 3, float> H;
  H(0, 0) = 0.0f;
  H(0, 1) = 0.0f;
  H(0, 2) = 1.0f;

  float measYaw = normalizeAngle(imuYaw);
  float predictedYaw = m_state(2, 0);
  float residual = normalizeAngle(measYaw - predictedYaw);

  float R = m_noise.rYaw;

  // H * P * H^T simplifies to P(2,2) since H = [0, 0, 1]
  BLA::Matrix<3, 1, float> PHt;
  PHt(0, 0) = m_P(0, 2);
  PHt(1, 0) = m_P(1, 2);
  PHt(2, 0) = m_P(2, 2);

  float S = m_P(2, 2) + R;

  // Prevent division by zero
  if (S < 1e-9f) {
    S = 1e-9f;
  }

  // Kalman gain (3x1 vector)
  BLA::Matrix<3, 1, float> K;
  K(0, 0) = PHt(0, 0) / S;
  K(1, 0) = PHt(1, 0) / S;
  K(2, 0) = PHt(2, 0) / S;

  m_state(0, 0) += K(0, 0) * residual;
  m_state(1, 0) += K(1, 0) * residual;
  m_state(2, 0) = normalizeAngle(m_state(2, 0) + K(2, 0) * residual);

  BLA::Matrix<3, 3, float> I = identityMatrix<3>();
  BLA::Matrix<3, 3, float> KH;
  
  // K * H where K is 3x1 and H is 1x3, result is 3x3
  KH(0, 0) = K(0, 0) * H(0, 0); KH(0, 1) = K(0, 0) * H(0, 1); KH(0, 2) = K(0, 0) * H(0, 2);
  KH(1, 0) = K(1, 0) * H(0, 0); KH(1, 1) = K(1, 0) * H(0, 1); KH(1, 2) = K(1, 0) * H(0, 2);
  KH(2, 0) = K(2, 0) * H(0, 0); KH(2, 1) = K(2, 0) * H(0, 1); KH(2, 2) = K(2, 0) * H(0, 2);

  m_P = (I - KH) * m_P;

  enforceSymmetry();
}

void Odometry::predictWithSlipDetection(float v, float omega, float imuOmega, float dt) {
  float omegaDiff = fabsf(omega - imuOmega);
  m_slipDetected = (omegaDiff > m_noise.slipThreshold);
  
  float effectiveV = v;
  float effectiveOmega = omega;
  
  if (m_slipDetected) {
    effectiveOmega = omega * (1.0f - m_noise.slipOmegaGain) 
                   + imuOmega * m_noise.slipOmegaGain;
    
    effectiveV = v * (1.0f - m_noise.slipLinearGain);
  }
  
  // Call standard predict with corrected values
  predict(effectiveV, effectiveOmega, dt);
  
  if (m_slipDetected) {
    // Add extra uncertainty to position bc we don't know where we really went
    m_P(0, 0) += m_noise.qX * 10.0f;  // 10x position uncertainty during slip
    m_P(1, 1) += m_noise.qY * 10.0f;
    // Heading is corrected by IMU in correct() step, so less worried there
  }
}

Pose Odometry::getPose() const {
  Rotation rot;
  rot.setFromRadians(0.0f, 0.0f, m_state(2, 0));
  return Pose(m_state(0, 0), m_state(1, 0), rot);
}

float Odometry::normalizeAngle(float angle) {
  while (angle > PI_F) {
    angle -= TWO_PI_F;
  }
  while (angle < -PI_F) {
    angle += TWO_PI_F;
  }
  return angle;
}

void Odometry::enforceSymmetry() {
  for (int r = 0; r < 3; ++r) {
    for (int c = r + 1; c < 3; ++c) {
      float avg = 0.5f * (m_P(r, c) + m_P(c, r));
      m_P(r, c) = avg;
      m_P(c, r) = avg;
    }
  }
}
