// #include <Arduino.h>
// #include <stm32f103xe.h>

// class Stepper {
//     private:
//   static Stepper *steppers[2];

//   PinName _stepPin;
//   PinName _dirPin;
//   PinName _enablePin;

//   int _speed; // steps/s
//   int _accel; // steps/s^2
//   int _jerk;  // steps/s^3

//   volatile long _steps;
//   volatile long _targetSteps;
//   volatile bool _isRunning;

//   bool _isEnabled;
//   bool _isReversed;
//   int  _pulseWidthUs = 1;
//   long _periodUs     = 0;

//   TIM_TypeDef   *_timerInst;
//   HardwareTimer *_timer;

//     public:
//   Stepper(PinName      step,
//           PinName      dir,
//           PinName      en,
//           TIM_TypeDef *timer,
//           bool         isReversed = false)
//     : _stepPin(step),
//       _dirPin(dir),
//       _enablePin(en),
//       _timerInst(timer),
//       _isReversed(isReversed) {
//     _timer = new HardwareTimer(timer);

//     if (timer == TIM1) {
//       _timer->attachInterrupt(ISR_0);
//     } else if (timer == TIM2) {
//       _timer->attachInterrupt(ISR_1);
//     } else {
//     }

//     _timer->setOverflow(1000, MICROSEC_FORMAT);
//     _timer->pause();
//   }

//   int setAccel(int newAccel) { return _accel = newAccel; }
//   int setSpeed(int newSpeed) { return _speed = newSpeed; }
//   int setJerk(int newJerk) { return _jerk = newJerk; }

//   bool setIsReversed(bool is) { return _isReversed = is; }

//   void setIsRunning(bool is) {
//     if (_isRunning == is) return;

//     _isRunning = is;

//     if (is)
//       enable();
//     else
//       disable();
//   }

//   bool isAtTargetPosition() { return _targetSteps == _steps; }

//   void beginMovement() {
//     if (isAtTargetPosition()) return;

//     enable();
//     _timer->resume();
//   }

//   long updatePeriod() {
//     if (isAtTargetPosition())
//       _periodUs = 0;
//     else
//       _periodUs = 625;

//     return _periodUs;
//   }

//   void computeMovement() {
//     if (isAtTargetPosition()) {
//       setIsRunning(false);
//       _timer->setOverflow(1000, MICROSEC_FORMAT);
//       _timer->pause();
//     } else {
//       setIsRunning(true);
//     }

//     updatePeriod();
//   }

//   long moveTo(long steps) { return _targetSteps = steps; }
//   long move(long steps) { return _targetSteps += steps; }

//   void enable() {
//     if (_isEnabled == true) return;
//     digitalWriteFast(_enablePin, LOW);
//     _isEnabled = true;
//   }
//   void disable() {
//     if (_isEnabled == false) return;
//     digitalWriteFast(_enablePin, HIGH);
//     _isEnabled = false;
//   }

//   void runMovement() {
//     // do one step
//     digitalWriteFast(_stepPin, 1);
//     delayMicroseconds(_pulseWidthUs);
//     digitalWriteFast(_stepPin, 0);

//     _steps += 1;
//   }

//   void callback() {
//     if (!isAtTargetPosition()) { return _timer->pause(); }

//     computeMovement();
//     runMovement();

//     _timer->setOverflow(_periodUs, MICROSEC_FORMAT);
//   };

//   static void ISR_0() { steppers[0]->callback(); }
//   static void ISR_1() { steppers[1]->callback(); }
// };