// #include "kalman.h"

// #include <math.h>

// using namespace BLA;

// namespace {
// template <int R, int C> void zeroMatrix(Matrix<R, C> &m) {
//   for (int r = 0; r < R; ++r) {
//     for (int c = 0; c < C; ++c) { m(r, c) = 0.0; }
//   }
// }

// template <int N> Matrix<N, N> identityMatrix() {
//   Matrix<N, N> mat;
//   zeroMatrix(mat);
//   for (int i = 0; i < N; ++i) { mat(i, i) = 1.0; }
//   return mat;
// }
// } // namespace

// Kalman::Kalman() {
//   zeroMatrix(m_state);
//   reset();
// }

// void Kalman::reset(const Pose &initialPose) {
//   zeroMatrix(m_state);
//   m_state(0, 0) = initialPose.x;
//   m_state(1, 0) = initialPose.y;
//   m_state(2, 0) = initialPose.rot.getRadians().z;

//   zeroMatrix(m_P);
//   m_P(0, 0) = 25.0;
//   m_P(1, 1) = 25.0;
//   m_P(2, 2) = 0.09;
// }

// void Kalman::setNoise(const KalmanNoiseConfig &cfg) { m_noise = cfg; }

// void Kalman::predict(long   deltaLeftSteps,
//                      long   deltaRightSteps,
//                      double dtSeconds) {
//   double dl = cnv_stepsToCM(deltaLeftSteps);
//   double dr = cnv_stepsToCM(deltaRightSteps);
//   predictCentimeters(dl, dr, dtSeconds);
// }

// void Kalman::predictCentimeters(double deltaLeftCm,
//                                 double deltaRightCm,
//                                 double dtSeconds) {
//   double dCenter  = 0.5 * (deltaLeftCm + deltaRightCm);
//   double dTheta   = (deltaRightCm - deltaLeftCm) / DT_TRACK_WIDTH_CM;
//   double theta0   = m_state(2, 0);
//   double thetaMid = theta0 + 0.5 * dTheta;

//   applyPrediction(dCenter, dTheta, theta0, thetaMid);
//   propagateCovariance(dCenter, dTheta, theta0, thetaMid, dtSeconds);
// }

// void Kalman::applyPrediction(double dCenterCm,
//                              double dThetaRad,
//                              double thetaPrior,
//                              double thetaMid) {
//   m_state(0, 0) += dCenterCm * cos(thetaMid);
//   m_state(1, 0) += dCenterCm * sin(thetaMid);
//   m_state(2, 0) = normalizeAngle(thetaPrior + dThetaRad);
// }

// void Kalman::propagateCovariance(double dCenterCm,
//                                  double dThetaRad,
//                                  double thetaPrior,
//                                  double thetaMid,
//                                  double dtSeconds) {
//   (void)thetaPrior;

//   Matrix<3, 3> F_mat = identityMatrix<3>();
//   F_mat(0, 2)        = -dCenterCm * sin(thetaMid);
//   F_mat(1, 2)        = dCenterCm * cos(thetaMid);

//   Matrix<3, 3> Q;
//   updateProcessNoise(dCenterCm, dThetaRad, dtSeconds, Q);

//   Matrix<3, 3> Ft = ~F_mat;
//   m_P             = F_mat * m_P * Ft + Q;
//   enforceSymmetry();
// }

// void Kalman::updateProcessNoise(double        dCenterCm,
//                                 double        dThetaRad,
//                                 double        dtSeconds,
//                                 Matrix<3, 3> &Q) const {
//   zeroMatrix(Q);

//   double travel = fabs(dCenterCm);
//   double turn   = fabs(dThetaRad);

//   double sigmaPos =
//     m_noise.processLinearStd + m_noise.processLinearStdPerCm * travel +
//     ((dtSeconds > 0.0) ? m_noise.processLinearStd * dtSeconds : 0.0);
//   double sigmaYaw =
//     m_noise.processAngularStd + m_noise.processAngularStdPerRad * turn +
//     ((dtSeconds > 0.0) ? m_noise.processAngularStd * dtSeconds : 0.0);

//   double varPos = sigmaPos * sigmaPos;
//   double varYaw = sigmaYaw * sigmaYaw;
//   if (varPos < 1e-9) varPos = 1e-9;
//   if (varYaw < 1e-9) varYaw = 1e-9;

//   Q(0, 0) = varPos;
//   Q(1, 1) = varPos;
//   Q(2, 2) = varYaw;
// }

// void Kalman::updateWithIMU(const RotationEuler &rotation) {
//   updateWithIMUYaw(rotation.z);
// }

// void Kalman::updateWithIMUYaw(double yawRadians) {
//   float measYaw  = normalizeAngle(yawRadians);
//   float residual = normalizeAngle(measYaw - m_state(2, 0));

//   Matrix<1, 3> H;
//   zeroMatrix(H);
//   H(0, 2) = 1.0;

//   Matrix<3, 1> PHt = m_P * (~H);
//   float        s   = (H * PHt)(0, 0) + (m_noise.imuYawStd * m_noise.imuYawStd);
//   if (s < 1e-9) s = 1e-9;

//   Matrix<3, 1> K = PHt * (1.0f / s);
//   m_state        = m_state + K * residual;
//   m_state(2, 0)  = normalizeAngle(m_state(2, 0));

//   Matrix<3, 3> I  = identityMatrix<3>();
//   Matrix<3, 3> KH = K * H;
//   m_P             = (I - KH) * m_P;
//   enforceSymmetry();
// }

// Pose Kalman::getPoseEstimate() const {
//   Rotation rot;
//   rot.setFromRadians(0.0, 0.0, m_state(2, 0));
//   return Pose(m_state(0, 0), m_state(1, 0), rot);
// }

// RotationEuler Kalman::getRotationEstimate() const {
//   return RotationEuler(0.0, 0.0, m_state(2, 0));
// }

// double Kalman::normalizeAngle(double angle) {
//   while (angle > PI) angle -= TWO_PI;
//   while (angle < -PI) angle += TWO_PI;
//   return angle;
// }

// void Kalman::enforceSymmetry() {
//   for (int r = 0; r < 3; ++r) {
//     for (int c = r + 1; c < 3; ++c) {
//       double avg = 0.5 * (m_P(r, c) + m_P(c, r));
//       m_P(r, c)  = avg;
//       m_P(c, r)  = avg;
//     }
//   }
// }
