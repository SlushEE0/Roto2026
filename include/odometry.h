#pragma once

#include <BasicLinearAlgebra.h>
#include <utils.h>

struct OdometryNoiseConfig {
  // Process noise (Q matrix diagonal elements)
  // These represent uncertainty ADDED during prediction step

  // Linear position noise: trust odometry reasonably well for distance
  float qX     = 0.01f;  // variance in x prediction (cm^2)
  float qY     = 0.01f;  // variance in y prediction (cm^2)

  // Angular noise: HIGH value because we DON'T trust wheel-based heading
  // The IMU will correct this in the update step
  float qTheta = 0.1f;   // variance in theta prediction (rad^2) - HIGH to defer to IMU

  // Measurement noise (R matrix)
  // This represents uncertainty in the IMU yaw measurement
  // VERY LOW value because we trust the BNO085 IMU almost exclusively for heading
  float rYaw   = 0.001f; // variance in IMU yaw measurement (rad^2) - LOW to trust IMU
  
  // Slip detection parameters
  // Compare wheel-derived omega with IMU gyro omega to detect wheel slip
  float slipThreshold = 0.3f;      // rad/s difference to trigger slip detection
  float slipOmegaGain = 0.9f;      // How much to trust IMU gyro during slip (0-1)
  float slipLinearGain = 0.5f;     // How much to reduce linear velocity trust during slip (0-1)
};

class Odometry {
public:
  Odometry();

  void reset(const Pose& pose = Pose());
  void setNoise(const OdometryNoiseConfig& cfg);


  void predict(float v, float omega, float dt);
  void predictWithSlipDetection(float v, float omega, float imuOmega, float dt);
  void correct(float imuYaw);

  bool isSlipDetected() const { return m_slipDetected; }

  Pose getPose() const;

  float getX() const     { return m_state(0, 0); }
  float getY() const     { return m_state(1, 0); }
  float getTheta() const { return m_state(2, 0); }

  const BLA::Matrix<3, 3, float>& getCovariance() const { return m_P; }

private:
  static float normalizeAngle(float angle);
  void enforceSymmetry();

  // State vector: [x, y, theta]^T
  BLA::Matrix<3, 1, float> m_state;

  // Covariance matrix (3x3)
  BLA::Matrix<3, 3, float> m_P;

  OdometryNoiseConfig m_noise;
  bool m_slipDetected;
};
