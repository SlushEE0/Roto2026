#pragma once

#include "bno.h"
#include "kalman.h"
#include "stepper.h"
#include "utils.h"

#include <cstddef>
#include <cstdint>

class DifferentialDrive {
    public:
  enum class Mode { Idle = 0, Velocity, Position };
  enum class PoseStage { Idle = 0, RotateToHeading, Translate, FinalRotate };

  DifferentialDrive(Stepper &left,
                    Stepper &right,
                    Kalman  *filter = nullptr,
                    BNO     *imu    = nullptr);

  void setFilter(Kalman *filter);
  void setIMU(BNO *imu);

  void resetPose(const Pose &pose = Pose());

  void commandVelocity(double linearCmPerSec, double angularDegPerSec);
  void commandWheelVelocities(double leftCmPerSec, double rightCmPerSec);

  void driveStraight(double distanceCm, double speedCmPerSec);
  void turnDegrees(double degrees, double speedDegPerSec);
  void moveToPose(const Pose &target,
                  double      linearSpeedCmPerSec,
                  double      turnSpeedDegPerSec);

  void stop(bool disableDrivers = false);

  void update();

  Pose          getPose() const { return _pose; }
  RotationEuler getRotation() const { return _pose.rot.getRadians(); }
  Mode          mode() const { return _mode; }
  bool          isBusy() const;
  double        headingHold() const { return _headingHold; }
  void          setHeadingHoldGain(double gain) { _headingGain = gain; }

    private:
  void          applyVelocityCommand(double leftCmPerSec, double rightCmPerSec);
  void          updatePose(long deltaLeft, long deltaRight, double dtSeconds);
  void          updatePoseSequence();
  void          refreshHeadingHold();
  double        currentYaw();
  static double degToRad(double deg);
  static double normalizeAngle(double angle);

  Stepper &_left;
  Stepper &_right;
  Kalman  *_filter;
  BNO     *_imu;

  Mode      _mode;
  PoseStage _poseStage;
  bool      _poseModeActive;

  Pose _pose;
  Pose _poseTarget;

  double _linearCmd;
  double _angularCmd;
  double _headingHold;
  double _headingGain;

  double _poseLinearSpeed;
  double _poseTurnSpeed;
  double _poseToleranceCm;
  double _poseToleranceRad;

  long _leftTarget;
  long _rightTarget;

  long _prevLeftSteps;
  long _prevRightSteps;

  float _positionSpeedSteps;

  unsigned long _lastUpdateMicros;
};
