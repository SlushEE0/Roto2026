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

struct Trajectory {
  const Pose* points;
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

  struct {
    float degrees;
    float speedDegPerSec;
  } turn;
  Pose targetPose;
  float linearSpeed;
  float turnSpeed;
  struct {
    const Pose* points;
    std::size_t count;
    float linearSpeed;
    float turnSpeed;
    std::size_t currentIndex; // To track progress within the trajectory
  } trajectory;

  DrivetrainCommand() : type(DrivetrainCommandType::Idle) {
    turn = {0.0f, 0.0f};
    targetPose = Pose();
    trajectory = {nullptr, 0, 0.0f, 0.0f, 0};
  }
};

class DifferentialDrive {
public:
  DifferentialDrive(Stepper& left,
                    Stepper& right,
                    Odometry* filter = nullptr,
                    BNO* imu = nullptr);

  void setFilter(Odometry* filter);
  void setIMU(BNO* imu);
  void setLinearPID(float kP, float kI, float kD);
  void setAngularPID(float kP, float kI, float kD);

  void resetPose(const Pose& pose = Pose());
  void stop(); // Clears queue and stops motors
  void update();
  bool isBusy() const;
  Pose getPose() const;
  float getHeading() const;

  DrivetrainCommandType getCurrentCommandType() const {
    return _activeCommand.type;
  }
  bool isExecuting() const {
    return _isExecuting;
  }
  std::size_t getQueueCount() const {
    return _queueCount;
  }
  const char* getSubStateName() const;

  bool queueTurnDegrees(float degrees, float speedDegPerSec);
  bool queueMoveToPose(const Pose& target, float linearSpeed, float turnSpeed);
  bool queueFollowTrajectory(const Trajectory& traj,
                             float linearSpeed,
                             float turnSpeed);

private:
  void processCommand();
  void handleTurnDegrees();
  void handleMoveToPose();
  void handleFollowTrajectory();

  void setWheelVelocities(float leftCmPerSec, float rightCmPerSec);
  float normalizeAngle(float angle);
  float degToRad(float deg);
  float radToDeg(float rad);

  Stepper& _left;
  Stepper& _right;
  Odometry* _filter;
  BNO* _imu;

  DrivetrainCommand _queue[kDrivetrainQueueSize];
  std::size_t _queueHead;
  std::size_t _queueTail;
  std::size_t _queueCount;

  DrivetrainCommand _activeCommand;
  bool _isExecuting;

  Pose _startPose;
  long _startLeftSteps;
  long _startRightSteps;
  float _targetHeading;

  PIDController _linearPID;  // For distance control
  PIDController _angularPID; // For heading/turning control

  enum class SubState {
    Init,
    AlignToTarget,
    DriveToTarget,
    FinalAlign,
    Done
  } _subState;
};
