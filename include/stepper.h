#pragma once
#include <Arduino.h>
#include <HardwareTimer.h>

class Stepper {
    public:
  Stepper(HardwareTimer *timer,
          uint8_t        stepPin,
          uint8_t        dirPin,
          uint8_t        enablePin,
          bool           invertDir = false);

  void begin(uint32_t timerFreqHz = 2000000UL);
  void enable();
  void disable();

  void moveTo(int32_t targetPosition) {
    moveTo(targetPosition, _maxSpeed, _acceleration);
  };
  void moveTo(int32_t targetPosition,
              int32_t maxSpeedStepsPerSec,
              int32_t accelStepsPerSec2);
  void moveBy(int32_t stepDelta) {
    moveBy(stepDelta, _maxSpeed, _acceleration);
  };
  void moveBy(int32_t stepDelta,
              int32_t maxSpeedStepsPerSec,
              int32_t accelStepsPerSec2) {
    moveTo(
      _currentPosition + stepDelta, maxSpeedStepsPerSec, accelStepsPerSec2);
  };
  void setTarget(int32_t targetPosition);
  void setMaxSpeed(float stepsPerSecond);
  void setAcceleration(float stepsPerSecondSquared);

  bool isBusy() const { return _running; }
  bool isAtTarget() const { return _currentPosition == _targetPosition; }

  int32_t currentPosition() const { return _currentPosition; }
  int32_t targetPosition() const { return _targetPosition; }
  float   currentSpeed() const { return _currentSpeed; }

    private:
  void handleTimerInterrupt();
  bool planNextStep();
  void applyDirection(int8_t dir);
  void primeTimer(uint32_t periodUs);

  HardwareTimer *_timer;
  uint8_t        _stepPin;
  uint8_t        _dirPin;
  uint8_t        _enablePin;
  bool           _invertDir;

  volatile int32_t _currentPosition;
  volatile int32_t _targetPosition;

  volatile float _currentSpeed; // signed steps/s
  volatile float _maxSpeed;
  volatile float _acceleration;

  volatile uint32_t _stepIntervalUs;
  volatile bool     _running;
  volatile bool     _stepPinIsHigh;
  volatile int8_t   _directionSign;
};
