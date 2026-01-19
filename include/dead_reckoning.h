#pragma once

#include "bno.h"
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

  // Get current heading from IMU (radians)
  float getHeading() const;

  // Command Queueing
  bool queueDriveStraight(float distanceCm, float speedCmPerSec);
  bool queueTurn(float degrees, float speedDegPerSec);

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
};
