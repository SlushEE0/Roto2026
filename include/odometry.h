#pragma once

#include <BasicLinearAlgebra.h>
#include <utils.h>

/**
 * @brief Noise configuration for the EKF odometry filter.
 *
 * TUNING NOTES:
 * - The IMU (BNO085) is much more accurate than wheel odometry for heading.
 * - We tune Q (process noise) to have HIGH angular uncertainty, so the filter
 *   doesn't trust the wheel-based heading prediction.
 * - We tune R (measurement noise) to have LOW angular uncertainty, so the filter
 *   heavily trusts the IMU yaw measurement.
 * - For linear position (x, y), we trust the wheel odometry since we have no
 *   absolute position sensor (GPS, etc.), so Q for x/y is relatively low.
 */
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

/**
 * @brief Loosely Coupled Extended Kalman Filter for 2D robot odometry.
 *
 * State vector: x = [x, y, theta]^T
 *   - x, y: position in centimeters
 *   - theta: heading in radians
 *
 * Prediction: Uses unicycle model with linear velocity (v) and angular velocity (omega)
 * Correction: Uses BNO085 IMU yaw angle to correct theta state
 *
 * IMPORTANT: All calculations use float (single precision) for STM32F103 compatibility
 * (no hardware FPU - double operations are emulated and very slow).
 */
class Odometry {
public:
  Odometry();

  /**
   * @brief Reset the filter to a known pose.
   * @param pose Initial pose (default: origin)
   */
  void reset(const Pose& pose = Pose());

  /**
   * @brief Configure the noise parameters for Q and R matrices.
   * @param cfg Noise configuration struct
   */
  void setNoise(const OdometryNoiseConfig& cfg);

  /**
   * @brief EKF Prediction step using unicycle motion model.
   *
   * Motion model (unicycle):
   *   x' = x + v * cos(theta) * dt
   *   y' = y + v * sin(theta) * dt
   *   theta' = theta + omega * dt
   *
   * @param v Linear velocity in cm/s (positive = forward)
   * @param omega Angular velocity in rad/s (positive = counter-clockwise)
   * @param dt Time step in seconds
   */
  void predict(float v, float omega, float dt);

  /**
   * @brief EKF Prediction with slip detection using IMU gyro.
   *
   * Compares wheel-derived angular velocity with IMU gyro to detect wheel slip.
   * If slip is detected, trusts IMU gyro for omega and reduces linear velocity.
   *
   * @param v Linear velocity from wheel odometry in cm/s
   * @param omega Angular velocity from wheel odometry in rad/s
   * @param imuOmega Angular velocity from IMU gyroscope in rad/s
   * @param dt Time step in seconds
   */
  void predictWithSlipDetection(float v, float omega, float imuOmega, float dt);

  /**
   * @brief Check if wheel slip was detected on last prediction.
   * @return true if slip was detected
   */
  bool isSlipDetected() const { return m_slipDetected; }

  /**
   * @brief EKF Correction step using IMU yaw measurement.
   *
   * Measurement model: z = theta (direct observation of heading)
   *
   * The IMU is trusted almost exclusively for heading correction.
   * See OdometryNoiseConfig tuning notes.
   *
   * @param imuYaw Yaw angle from IMU in radians
   */
  void correct(float imuYaw);

  /**
   * @brief Get the current pose estimate.
   * @return Pose struct with x, y (cm) and rotation
   */
  Pose getPose() const;

  /**
   * @brief Get individual state components.
   */
  float getX() const     { return m_state(0, 0); }
  float getY() const     { return m_state(1, 0); }
  float getTheta() const { return m_state(2, 0); }

  /**
   * @brief Get the covariance matrix (for diagnostics).
   */
  const BLA::Matrix<3, 3, float>& getCovariance() const { return m_P; }

private:
  /**
   * @brief Normalize angle to [-PI, PI] range.
   *
   * This is CRITICAL for the correction step to handle the wrap-around
   * problem (e.g., when actual angle is -179° and measurement is +179°).
   *
   * @param angle Angle in radians
   * @return Normalized angle in [-PI, PI]
   */
  static float normalizeAngle(float angle);

  /**
   * @brief Enforce symmetry in covariance matrix (numerical stability).
   */
  void enforceSymmetry();

  // State vector: [x, y, theta]^T
  BLA::Matrix<3, 1, float> m_state;

  // Covariance matrix (3x3)
  BLA::Matrix<3, 3, float> m_P;

  // Noise configuration
  OdometryNoiseConfig m_noise;
  
  // Slip detection state
  bool m_slipDetected;
};
