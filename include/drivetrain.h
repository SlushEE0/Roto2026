#pragma once

#include "bno.h"
#include "pid.h"
#include "stepper.h"
#include "utils.h"

#include <cstddef>
#include <cstdint>

// Maximum number of commands in the queue
static constexpr std::size_t kDrivetrainQueueSize = 64;

// ── Command types ──────────────────────────────────────────────────────────
enum class CommandType { Idle, DriveStraight, Turn };

struct DriveCommand {
  CommandType type;

  float distance;    // cm  (DriveStraight only)
  float linearSpeed; // cm/s
  float angleDeg;    // degrees (Turn only, positive = CCW)
  float turnSpeed;   // deg/s

  DriveCommand()
      : type(CommandType::Idle), distance(0), linearSpeed(0), angleDeg(0),
        turnSpeed(0) {
  }
};

// ── DifferentialDrive ──────────────────────────────────────────────────────
// Simple differential-drive controller.
//   • DriveStraight: uses step counting for distance, IMU for heading hold.
//   • Turn: uses IMU heading to rotate in place.
//   • All commands go through a FIFO queue.
//   • setTimeTarget() auto-calculates speeds for speed-less queued commands.
class DifferentialDrive {
public:
  DifferentialDrive(Stepper& left, Stepper& right, BNO* imu = nullptr);

  // ── Configuration ──────────────────────────────────────────────────────
  void setIMU(BNO* imu);
  void setHeadingPID(float kP, float kI, float kD);
  void setTurnPID(float kP, float kI, float kD);

  // ── Control ────────────────────────────────────────────────────────────
  void stop();   // Clear queue, stop motors
  void update(); // Call every loop iteration
  bool isBusy() const;

  // ── State queries ──────────────────────────────────────────────────────
  float getHeading() const; // radians, from IMU
  std::size_t getQueueCount() const {
    return _queueCount;
  }
  bool isExecuting() const {
    return _isExecuting;
  }

  // ── Queue commands (with explicit speeds) ──────────────────────────────
  bool queueDriveStraight(float distanceCm, float speedCmPerSec);
  bool queueTurnDegrees(float deg, float speedDegPerSec);

  // ── Queue commands (speed-less, for use with setTimeTarget) ────────────
  bool queueDrive(float distanceCm);
  bool queueTurn(float deg);

  // ── Time-target speed resolver ─────────────────────────────────────────
  // After queuing speed-less commands, call this to auto-calculate speeds
  // so the entire sequence completes in |totalSeconds|.
  // Commands with explicit speeds have their estimated time subtracted
  // from the budget first; the remainder is distributed to speed-less ones.
  bool setTimeTarget(float totalSeconds);

private:
  void processCommand();
  void handleDriveStraight();
  void handleTurn();

  void setWheelVelocities(float leftCmPerSec, float rightCmPerSec);
  float normalizeAngle(float angle) const;
  float degToRad(float deg) const;
  float radToDeg(float rad) const;

  // Hardware
  Stepper& _left;
  Stepper& _right;
  BNO* _imu;

  // Queue
  DriveCommand _queue[kDrivetrainQueueSize];
  std::size_t _queueHead;
  std::size_t _queueTail;
  std::size_t _queueCount;

  // Active command
  DriveCommand _activeCommand;
  bool _isExecuting;

  // Motion state
  long _startLeftSteps;
  long _startRightSteps;
  float _startHeading;
  float _targetHeading;

  // PID controllers
  PIDController _headingPID; // heading correction while driving straight
  PIDController _turnPID;    // turning in place
};
