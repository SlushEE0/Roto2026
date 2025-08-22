#pragma once

#include <Arduino.h>

class Stepper {
public:
  Stepper(uint8_t stepPin, uint8_t dirPin, uint8_t enablePin = 255);

  void setSpeed(long stepsPerSec); // set constant speed
  void moveTo(long absolute); // set target absolute position
  void move(long relative); // relative move
  long currentPosition(); // get current steps
  bool isBusy(); // still moving?

  void enable();
  void disable();

  // internal: called from ISR
  void stepService();

private:
  uint8_t _stepPin, _dirPin, _enablePin;
  volatile long _targetPos = 0;
  volatile long _currentPos = 0;
  volatile unsigned long _stepInterval = 1000; // µs
  volatile unsigned long _lastStepMicros = 0;
  volatile bool _dir = true;

  friend void stepperISR(); // allow ISR access
};
