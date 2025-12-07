#pragma once

#include "bno.h"
#include "kalman.h"
#include "stepper.h"
#include "utils.h"

#include <cstddef>
#include <cstdint>

// Maximum number of commands in the queue
static constexpr std::size_t kDrivetrainQueueSize = 16;

// Trajectory is a sequence of poses
struct Trajectory {
  const Pose *points;
  std::size_t count;
};

// Command types
enum class DrivetrainCommandType {
  Idle,
  DriveStraight,
  TurnDegrees,
  MoveToPose,
  FollowTrajectory
};

// Unified command structure
struct DrivetrainCommand {
  DrivetrainCommandType type;
  union {
    struct {
      double distCm;
      double speedCmPerSec;
    } straight;
    struct {
      double degrees;
      double speedDegPerSec;
    } turn;
    struct {
      Pose   target;
      double linearSpeed;
      double turnSpeed;
    } pose;
    struct {
      const Pose *points;
      std::size_t count;
      double      linearSpeed;
      double      turnSpeed;
      std::size_t currentIndex; // To track progress within the trajectory
    } trajectory;
  } data;

  DrivetrainCommand() : type(DrivetrainCommandType::Idle) {}
};

class DifferentialDrive {
    public:
  DifferentialDrive(Stepper &left,
                    Stepper &right,
                    Kalman  *filter = nullptr,
                    BNO     *imu    = nullptr);

  // Configuration
  void setFilter(Kalman *filter);
  void setIMU(BNO *imu);
  void setGains(double headingGain, double distanceGain);

  // State Management
  void   resetPose(const Pose &pose = Pose());
  void   stop(); // Clears queue and stops motors
  void   update();
  bool   isBusy() const;
  Pose   getPose() const;
  double getHeading() const;

  // Command Queueing
  bool queueDriveStraight(double distanceCm, double speedCmPerSec);
  bool queueTurnDegrees(double degrees, double speedDegPerSec);
  bool queueMoveToPose(const Pose &target,
                       double      linearSpeed,
                       double      turnSpeed);
  bool queueFollowTrajectory(const Trajectory &traj,
                             double            linearSpeed,
                             double            turnSpeed);

    private:
  // Control Loop Handlers
  void processCommand();
  void handleDriveStraight();
  void handleTurnDegrees();
  void handleMoveToPose();
  void handleFollowTrajectory();

  // Low-level helpers
  void   setWheelVelocities(double leftCmPerSec, double rightCmPerSec);
  double getDistanceTraveled(long startLeft,
                             long startRight,
                             long currentLeft,
                             long currentRight);
  double normalizeAngle(double angle);
  double degToRad(double deg);
  double radToDeg(double rad);

  // Member Variables
  Stepper &_left;
  Stepper &_right;
  Kalman  *_filter;
  BNO     *_imu;

  // Queue
  DrivetrainCommand _queue[kDrivetrainQueueSize];
  std::size_t       _queueHead;
  std::size_t       _queueTail;
  std::size_t       _queueCount;

  // Active Command State
  DrivetrainCommand _activeCommand;
  bool              _isExecuting;

  // Control State
  Pose   _startPose;
  long   _startLeftSteps;
  long   _startRightSteps;
  double _targetHeading;
  
  // Gains
  double _headingGain;  // For straight driving correction
  double _turnGain;     // For turning control
  
  // Internal sub-state for complex moves (MoveToPose)
  enum class SubState {
      Init,
      AlignToTarget,
      DriveToTarget,
      FinalAlign,
      Done
  } _subState;
};
