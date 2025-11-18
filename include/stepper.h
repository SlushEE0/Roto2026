#pragma once

#include <Arduino.h>
#include <HardwareTimer.h>
#include <math.h>

class Stepper {
    public:
  enum class Mode : uint8_t { Idle = 0, Position, Velocity };

  Stepper(HardwareTimer *timer,
          uint8_t        stepPin,
          uint8_t        dirPin,
          uint8_t        enablePin,
          bool           invertDir = false);

  void begin(uint32_t timerFreqHz = 2000000UL);
  void enable();
  void disable();
  void stop(bool disableDriver = false);

  void commandPosition(int32_t targetPosition, float speedStepsPerSec = 0.0f);
  void commandRelative(int32_t stepDelta, float speedStepsPerSec = 0.0f);
  void commandVelocity(float stepsPerSecond);

  void moveTo(int32_t targetPosition) {
    commandPosition(targetPosition, _maxSpeed);
  }
  void moveTo(int32_t targetPosition,
              int32_t maxSpeedStepsPerSec,
              int32_t /*accelStepsPerSec2*/) {
    commandPosition(targetPosition,
                    fabsf(static_cast<float>(maxSpeedStepsPerSec)));
  }
  void moveBy(int32_t stepDelta) { commandRelative(stepDelta, _maxSpeed); }
  void moveBy(int32_t stepDelta,
              int32_t maxSpeedStepsPerSec,
              int32_t /*accelStepsPerSec2*/) {
    commandRelative(stepDelta, fabsf(static_cast<float>(maxSpeedStepsPerSec)));
  }

  void setTarget(int32_t targetPosition) {
    commandPosition(targetPosition, _maxSpeed);
  }
  void setMaxSpeed(float stepsPerSecond);
  void setAcceleration(float stepsPerSecondSquared) {
    _acceleration = stepsPerSecondSquared;
  }

  Mode mode() const { return static_cast<Mode>(_mode); }
  bool isBusy() const { return _running; }
  bool isAtTarget() const {
    return (_mode == static_cast<uint8_t>(Mode::Position)) &&
           (_currentPosition == _targetPosition);
  }

  int32_t currentPosition() const { return _currentPosition; }
  int32_t targetPosition() const { return _targetPosition; }
  float   currentSpeed() const { return fabsf(_commandVelocity); }
  float   commandedVelocity() const { return _commandVelocity; }
  float   maxSpeed() const { return _maxSpeed; }
  float   acceleration() const { return _acceleration; }

    private:
  void     handleTimerInterrupt();
  void     applyDirection(int8_t dir);
  void     primeTimer(uint32_t periodUs);
  uint32_t intervalFromSpeed(float stepsPerSecond) const;
  void     hardStopFromISR();

  HardwareTimer *_timer;
  uint8_t        _stepPin;
  uint8_t        _dirPin;
  uint8_t        _enablePin;
  bool           _invertDir;

  volatile int32_t _currentPosition;
  volatile int32_t _targetPosition;

  volatile float _commandVelocity; // signed steps/s
  float          _maxSpeed;
  float          _acceleration;

  volatile uint32_t _stepIntervalUs;
  volatile bool     _running;
  volatile bool     _stepPinIsHigh;
  volatile int8_t   _directionSign;
  volatile uint8_t  _mode;
};
