#pragma once

#include "bno.h"
#include "kalman.h"
#include "stepper.h"
#include "utils.h"

#include <cstddef>
#include <deque>

class DifferentialDrive {
    public:
  enum class Mode { Idle = 0, DrivingStraight, Turning, MoveToPose };
  enum class SubAction { None = 0, Driving, Turning };
  enum class PosePhase { None = 0, RotateToHeading, DriveStraight, FinalTurn };
  enum class MoveType { DriveStraight = 0, Turn, ToPose };

  struct MoveCommand {
    MoveType type     = MoveType::DriveStraight;
    double   distance = 0.0; // cm
    double   degrees  = 0.0; // deg
    Pose     targetPose;
    double   linearMax = 0.0; // cm/s
    double   linearAcc = 0.0; // cm/s^2
    double   turnMax   = 0.0; // deg/s
    double   turnAcc   = 0.0; // deg/s^2
  };

  DifferentialDrive(Stepper &left,
                    Stepper &right,
                    Kalman  *filter,
                    BNO     *imu = nullptr);

  void setFilter(Kalman *filter);
  void setIMU(BNO *imu);

  void resetPose(const Pose &pose = Pose());

  void queueDriveStraight(double distanceCm,
                          double maxSpeedCmPerSec = 0.0,
                          double accelCmPerSec2   = 0.0);

  void queueTurn(double degrees,
                 double maxSpeedDegPerSec = 0.0,
                 double accelDegPerSec2   = 0.0);

  void queueMoveToPose(const Pose &target,
                       double      linearSpeedCmPerSec,
                       double      accelCmPerSec2,
                       double      turnSpeedDegPerSec,
                       double      turnAccelDegPerSec2);

  void clearQueue();

  void stop();

  void update();

  Pose          getPose() const { return _pose; }
  RotationEuler getRotation() const { return _pose.rot.getRadians(); }
  Mode          mode() const { return _mode; }
  bool          isBusy() const;
  size_t        queuedMoves() const;

    private:
  Stepper &_left;
  Stepper &_right;
  Kalman  *_filter;
  BNO     *_imu;

  Mode      _mode;
  SubAction _subAction;
  PosePhase _posePhase;
  bool      _poseSequenceActive;
  bool      _hasActiveCommand;

  Pose _pose;

  MoveCommand             _activeCommand;
  std::deque<MoveCommand> _queue;

  Pose   _poseTarget;
  double _poseTargetHeading;
  double _poseFinalYaw;

  long _prevLeftSteps;
  long _prevRightSteps;

  long _baseLeftTarget;
  long _baseRightTarget;

  unsigned long _lastUpdateMicros;

  double _turnMaxSpeedDegPerSec;
  double _turnAccelDegPerSec2;
  double _linearMaxSpeedCmPerSec;
  double _linearAccelCmPerSec2;
  double _headingHold;

  void applyHeadingCorrection();
  void updatePoseSequence(bool leftBusy, bool rightBusy);
  void updateFilter(long deltaLeft, long deltaRight, double dtSeconds);
  void startNextCommand();
  bool
  startDrive(double distanceCm, double maxSpeedCmPerSec, double accelCmPerSec2);
  bool
  startTurn(double degrees, double maxSpeedDegPerSec, double accelDegPerSec2);
  bool          startPoseCommand(const MoveCommand &command);
  void          issueDriveCommand(long    stepDelta,
                                  int32_t maxSpeedSteps,
                                  int32_t accelSteps,
                                  Mode    commandMode);
  void          issueTurnCommand(long    stepDelta,
                                 int32_t maxSpeedSteps,
                                 int32_t accelSteps,
                                 Mode    commandMode);
  void          finishActiveCommand();
  double        currentYaw() const;
  static double normalizeAngle(double angle);
  static long   cmToSteps(double cm);
  static int32_t
  clampSpeedSteps(double requested, double defaultValue, int32_t minValue = 1);
};
