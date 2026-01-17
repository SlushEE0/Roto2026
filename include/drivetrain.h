#pragma once

#include "bno.h"
#include "odometry.h"
#include "pid.h"
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
  TurnDegrees,
  MoveToPose,
  FollowTrajectory
};

// Unified command structure
struct DrivetrainCommand {
  DrivetrainCommandType type;
  
  // Using separate structs instead of union to avoid non-trivial constructor issues
  // Only one of these is valid at a time, determined by 'type'
  struct {
    float degrees;
    float speedDegPerSec;
  } turn;
  struct {
    float  targetX;
    float  targetY;
    float  targetYaw;
    float  linearSpeed;
    float  turnSpeed;
  } pose;
  struct {
    const Pose *points;
    std::size_t count;
    float       linearSpeed;
    float       turnSpeed;
    std::size_t currentIndex; // To track progress within the trajectory
  } trajectory;

  DrivetrainCommand() : type(DrivetrainCommandType::Idle) {
    turn = {0.0f, 0.0f};
    pose = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    trajectory = {nullptr, 0, 0.0f, 0.0f, 0};
  }
};

class DifferentialDrive {
    public:
  DifferentialDrive(Stepper  &left,
                    Stepper  &right,
                    Odometry *filter = nullptr,
                    BNO      *imu    = nullptr);

  // Configuration
  void setFilter(Odometry *filter);
  void setIMU(BNO *imu);
  void setLinearPID(float kP, float kI, float kD);
  void setAngularPID(float kP, float kI, float kD);

  // State Management
  void  resetPose(const Pose &pose = Pose());
  void  stop(); // Clears queue and stops motors
  void  update();
  bool  isBusy() const;
  Pose  getPose() const;
  float getHeading() const;

  // Command Queueing
  bool queueTurnDegrees(float degrees, float speedDegPerSec);
  bool queueMoveToPose(const Pose &target,
                       float       linearSpeed,
                       float       turnSpeed);
  bool queueFollowTrajectory(const Trajectory &traj,
                             float             linearSpeed,
                             float             turnSpeed);

    private:
  // Control Loop Handlers
  void processCommand();
  void handleTurnDegrees();
  void handleMoveToPose();
  void handleFollowTrajectory();

  // Low-level helpers
  void  setWheelVelocities(float leftCmPerSec, float rightCmPerSec);
  float normalizeAngle(float angle);
  float degToRad(float deg);
  float radToDeg(float rad);

  // Member Variables
  Stepper  &_left;
  Stepper  &_right;
  Odometry *_filter;
  BNO      *_imu;

  // Queue
  DrivetrainCommand _queue[kDrivetrainQueueSize];
  std::size_t       _queueHead;
  std::size_t       _queueTail;
  std::size_t       _queueCount;

  // Active Command State
  DrivetrainCommand _activeCommand;
  bool              _isExecuting;

  // Control State
  Pose  _startPose;
  long  _startLeftSteps;
  long  _startRightSteps;
  float _targetHeading;
  
  // PID Controllers
  PIDController _linearPID;   // For distance control
  PIDController _angularPID;  // For heading/turning control
  
  // Internal sub-state for complex moves (MoveToPose)
  enum class SubState {
      Init,
      AlignToTarget,
      DriveToTarget,
      FinalAlign,
      Done
  } _subState;
};
