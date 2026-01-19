// #pragma once

// #include <BasicLinearAlgebra.h>
// #include <utils.h>



// struct KalmanNoiseConfig {
//   double processLinearStd        = 0.5;  // cm base noise
//   double processLinearStdPerCm   = 0.02; // cm noise growth per cm traveled
//   double processAngularStd       = 0.02; // rad base noise
//   double processAngularStdPerRad = 0.02; // rad noise growth per rad turned
//   double imuYawStd               = 0.01; // rad measurement noise
// };

// class Kalman {
//     public:
//   Kalman();

//   void reset(const Pose &initialPose = Pose());
//   void setNoise(const KalmanNoiseConfig &cfg);

//   void
//   predict(long deltaLeftSteps, long deltaRightSteps, double dtSeconds = 0.0);
//   void predictCentimeters(double deltaLeftCm,
//                           double deltaRightCm,
//                           double dtSeconds = 0.0);

//   void updateWithIMU(const RotationEuler &rotation);
//   void updateWithIMUYaw(double yawRadians);

//   Pose          getPoseEstimate() const;
//   RotationEuler getRotationEstimate() const;

//   double getX() const { return m_state(0, 0); }
//   double getY() const { return m_state(1, 0); }
//   double getYaw() const { return m_state(2, 0); }

//   const BLA::Matrix<3, 3> &covariance() const { return m_P; }

//     private:
//   static double normalizeAngle(double angle);
//   void          applyPrediction(double dCenterCm,
//                                 double dThetaRad,
//                                 double thetaPrior,
//                                 double thetaMid);
//   void          propagateCovariance(double dCenterCm,
//                                     double dThetaRad,
//                                     double thetaPrior,
//                                     double thetaMid,
//                                     double dtSeconds);
//   void          updateProcessNoise(double             dCenterCm,
//                                    double             dThetaRad,
//                                    double             dtSeconds,
//                                    BLA::Matrix<3, 3> &Q) const;
//   void          enforceSymmetry();

//   BLA::Matrix<3, 1> m_state;
//   BLA::Matrix<3, 3> m_P;
//   KalmanNoiseConfig m_noise;
// };
