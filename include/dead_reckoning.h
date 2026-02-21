#pragma once

#include "bno.h"
#include "pid.h"
#include "stepper.h"
#include "utils.h"

#include <cstddef>
#include <cstdint>

// Maximum number of commands in the queue
static constexpr std::size_t kDeadReckoningQueueSize = 16;

// Command types
enum class DeadReckoningCommandType { Idle, DriveStraight, Turn };

// Command structure
struct DeadReckoningCommand {
  DeadReckoningCommandType type;

  union {
    struct {
      float distanceCm;
      float speedCmPerSec;
    } straight;
    struct {
      float degrees;
      float speedDegPerSec;
    } turn;
  };

  DeadReckoningCommand() : type(DeadReckoningCommandType::Idle) {
    straight = {0.0f, 0.0f};
  }
};

class DeadReckoningDrivetrain {
public:
  DeadReckoningDrivetrain(Stepper& left, Stepper& right, BNO* imu = nullptr);

  // Configuration
  void setIMU(BNO* imu);
  void
  setHeadingGain(float gain);   // P-gain for heading correction during straight
  void setTurnGain(float gain); // P-gain for turn control

  // State Management
  void stop();         // Clears queue and stops motors
  void update();       // Call this in your main loop
  bool isBusy() const; // Returns true if executing or queue not empty
  std::size_t getQueueCount() const { return _queueCount; }

  // Get current heading from IMU (radians)
  float getHeading() const;

  // Command Queueing
  bool queueDriveStraight(float distanceCm, float speedCmPerSec);
  bool queueTurn(float degrees, float speedDegPerSec);

  // ── Ziegler-Nichols Turn Tuner ─────────────────────────────────────────
  // Blocks while it runs.  Logs Ku, Tu and computed PID gains to Serial1.
  // targetDeg   – amplitude of the test oscillation (e.g. 30°)
  // maxSpeedDeg – speed cap during tuning (deg/s)
  // startKp     – initial proportional gain to try
  // kpStep      – how much to increase Kp each iteration
  // maxKp       – safety cap
  void runZieglerNicholsTurnTune(float targetDeg = 45.0f,
                                 float maxSpeedDeg = 120.0f,
                                 float startKp = 0.5f,
                                 float kpStep = 0.25f,
                                 float maxKp = 20.0f);

  // Apply PID gains computed by the tuner (or manually chosen)
  void setTurnPID(float kP, float kI, float kD);
  const PIDController::Gains& getTurnPIDGains() const { return _turnPID.getGains(); }

private:
  // Command handlers
  void processCommand();
  void handleDriveStraight();
  void handleTurn();

  // Helpers
  void setWheelVelocities(float leftCmPerSec, float rightCmPerSec);
  float normalizeAngle(float angle);
  float degToRad(float deg);
  float radToDeg(float rad);

  // Hardware references
  Stepper& _left;
  Stepper& _right;
  BNO* _imu;

  // Command queue
  DeadReckoningCommand _queue[kDeadReckoningQueueSize];
  std::size_t _queueHead;
  std::size_t _queueTail;
  std::size_t _queueCount;

  // Active command state
  DeadReckoningCommand _activeCommand;
  bool _isExecuting;

  // Control state
  long _startLeftSteps;
  long _startRightSteps;
  float _startHeading; 
  float _targetHeading;

  float _headingGain;
  float _turnGain;

  // PID turn controller (replaces bare P when tuned)
  PIDController _turnPID;
  bool _useTurnPID = false;
  uint32_t _lastTurnUpdateMs = 0;
};
