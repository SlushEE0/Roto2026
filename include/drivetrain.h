#pragma once

#include "bno.h"
#include "kalman.h"
#include "stepper.h"
#include "utils.h"

class DifferentialDrive {
    public:
  enum class Mode { Idle = 0, DrivingStraight, Turning, MoveToPose };
  enum class SubAction { None = 0, Driving, Turning };

  DifferentialDrive(Stepper &left,
                    Stepper &right,
                    Kalman  *filter,
                    BNO     *imu = nullptr);

  void setFilter(Kalman *filter);
  void setIMU(BNO *imu);

  void resetPose(const Pose &pose = Pose());

  void driveStraight(double distanceCm,
                     double maxSpeedCmPerSec = 0.0,
                     double accelCmPerSec2   = 0.0);
  void driveDistance(double distanceCm,
                     double maxSpeedCmPerSec = 0.0,
                     double accelCmPerSec2   = 0.0) {
    driveStraight(distanceCm, maxSpeedCmPerSec, accelCmPerSec2);
  }

  void turnInPlace(double degrees,
                   double maxSpeedDegPerSec = 0.0,
                   double accelDegPerSec2   = 0.0);

  void moveToPose(Pose &target,
                  double      linearSpeedCmPerSec,
                  double      accelCmPerSec2,
                  double      turnSpeedDegPerSec,
                  double      turnAccelDegPerSec2);

  void stop();

  void update();

  Pose          getPose() const { return m_pose; }
  RotationEuler getRotation() const { return m_pose.rot.getRadians(); }
  Mode          mode() const { return m_mode; }
  bool          isBusy() const;

    private:
  enum class MovePhase { None = 0, RotateToHeading, DriveStraight, FinalTurn };

  Stepper &m_left;
  Stepper &m_right;
  Kalman  *m_filter;
  BNO     *m_imu;

  Mode      m_mode;
  SubAction m_subAction;
  MovePhase m_movePhase;
  bool      m_sequenceActive;

  Pose   m_pose;
  Pose   m_moveTargetPose;
  double m_headingHold;
  double m_targetYaw;
  double m_moveTargetHeading;
  double m_moveFinalYaw;

  long m_prevLeftSteps;
  long m_prevRightSteps;

  long m_baseLeftTarget;
  long m_baseRightTarget;

  unsigned long m_lastUpdateMicros;

  double m_turnMaxSpeedDegPerSec;
  double m_turnAccelDegPerSec2;
  double m_linearMaxSpeedCmPerSec;
  double m_linearAccelCmPerSec2;

  void          applyHeadingCorrection();
  void          handleMoveSequence(bool leftBusy, bool rightBusy);
  void          updateFilter(long deltaLeft, long deltaRight, double dtSeconds);
  double        currentYaw() const;
  static double normalizeAngle(double angle);
  static long   cmToSteps(double cm);
  static long   degToSteps(double degrees);
  static int32_t
  clampSpeedSteps(double requested, double defaultValue, int32_t minValue = 1);
  void commandDrive(long    stepDelta,
                    int32_t maxSpeedSteps,
                    int32_t accelSteps,
                    bool    preserveSequence);
  void commandTurn(double degrees,
                   double maxSpeedDegPerSec,
                   double accelDegPerSec2,
                   bool   preserveSequence);
};
