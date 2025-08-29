#pragma once
#include <Arduino.h>
#include <HardwareTimer.h>

class Stepper {
public:
  // timer: TIM1, TIM2, TIM3, TIM4 (use one timer instance per Stepper)
  Stepper(uint8_t stepPin,
          uint8_t dirPin,
          TIM_TypeDef *timer,
          int8_t enablePin = -1,
          bool enableActiveHigh = true,
          uint32_t pulse_us = 3);

  void setMaxSpeed(double steps_per_sec);
  void setAcceleration(double steps_per_sec2);
  void setSCurve(bool on);
  void setReverse(bool rev);
  void setPulseWidthUs(uint32_t pulse_us);

  // motion APIs
  void move(long steps); // relative
  void moveTo(long pos); // absolute
  void stop(); // graceful stop (finish current step)

  bool isRunning() const { return _running; }

  long currentPosition() const { return _currentPos; }

  long targetPosition() const { return _targetPos; }

  // start/stop timer manually (rarely needed)
  void startTimer();
  void stopTimer();
  void onTimerISR();

private:
  // speed/period computation (called from ISR, short)
  void computeNewPeriod();

  // pin & timer
  uint8_t _stepPin, _dirPin;
  int8_t _enablePin;
  bool _enableActiveHigh;
  bool _reverseDir;

  TIM_TypeDef *_timInstance;
  HardwareTimer *_hwTimer;

  // motion state (mostly volatile as updated in ISR)
  volatile long _currentPos = 0;
  volatile long _targetPos = 0;
  volatile bool _running = false;

  // profile
  double _maxSpeed = 2000.0f; // steps/sec
  double _accel = 500.0f; // steps/sec^2
  bool _sCurve = true;
  double _jerkFactor = 0.2f;

  // live speed -> converted to period
  double _currentSpeed = 0.0f; // steps/sec (signed)
  double _targetSpeed = 0.0f; // steps/sec (signed)
  volatile uint32_t _period_us = 1000; // full step period in microseconds
  uint32_t _pulse_us = 3; // pulse high time in microseconds
  uint32_t _min_period_us = 20; // clamp minimum period (max freq)
  volatile uint32_t _last_period_us = 1000;
  volatile uint8_t _phase =
    0; // 0 = waiting->start step (next interrupt will raise step pin), 1 =
       // pulse high (next will lower)

  // helper: map timer instance -> index
  static int timerIndex(TIM_TypeDef *tim);
};
