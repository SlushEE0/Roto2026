#include <Arduino.h>
#include <stm32f103xe.h>

class Stepper {
    private:
  static Stepper *steppers[2];
  int             _index;

  PinName _stepPin;
  PinName _dirPin;
  PinName _enablePin;

  int _speed = 3000; // steps/s
  int _accel = 1000; // steps/s^2
  int _jerk  = 9;    // steps/s^3

  volatile long _steps       = 0;
  volatile long _targetSteps = 0;
  volatile bool _isRunning   = false;

  int  _direction    = 1;
  bool _isEnabled    = false;
  bool _isReversed   = false;
  int  _pulseWidthUs = 1;
  long _periodUs     = 0;

  TIM_TypeDef   *_timerInst;
  HardwareTimer *_timer;

    public:
  Stepper(PinName      step,
          PinName      dir,
          PinName      en,
          TIM_TypeDef *timer,
          bool         isReversed = false)
    : _stepPin(step),
      _dirPin(dir),
      _enablePin(en),
      _timerInst(timer),
      _isReversed(isReversed) {
    _timer = new HardwareTimer(timer);

    if (timer == TIM1) {
      _index = 0;
    } else if (timer == TIM2) {
      _index = 1;
    } else {
      _index = -1; // Unsupported timer
    }

    switch (_index) {
      case 0:
        steppers[0] = this;
        _timer->attachInterrupt(ISR_0);
        break;
      case 1:
        steppers[1] = this;
        _timer->attachInterrupt(ISR_1);
        break;
      default:
        // Unsupported timer
        break;
    }

    _timer->setOverflow(1000, MICROSEC_FORMAT);
    _timer->pause();
  }

  int setAccel(int newAccel) { return _accel = newAccel; }
  int setSpeed(int newSpeed) { return _speed = newSpeed; }
  int setJerk(int newJerk) { return _jerk = newJerk; }

  bool setIsReversed(bool is) { return _isReversed = is; }
  void setIsRunning(bool is) {
    if (_isRunning == is) return;

    _isRunning = is;

    if (is)
      enable();
    else
      disable();
  }

  long getSteps() { return _steps; }
  long getTargetSteps() { return _targetSteps; }
  bool getIsEnabled() { return _isEnabled; }

  bool isAtTargetSteps() { return _targetSteps == _steps; }

  void beginMovement() {
    if (isAtTargetSteps()) return;

    enable();
    _timer->resume();
  }

  long updatePeriod() {
    if (isAtTargetSteps())
      _periodUs = 0;
    else
      _periodUs = 625;

    return _periodUs;
  }

  void computeMovement() {
    if (isAtTargetSteps()) {
      setIsRunning(false);
      _timer->setOverflow(1000, MICROSEC_FORMAT);
      _timer->pause();
    } else {
      setIsRunning(true);
    }

    if (_targetSteps > _steps) {
      _direction = 1;
    } else if (_targetSteps < _steps) {
      _direction = -1;
    }

    updatePeriod();
  }

  long moveTo(long steps) {
    _targetSteps = steps;
    beginMovement();

    return _targetSteps;
  }
  long move(long steps) {
    _targetSteps += steps;
    beginMovement();

    return _targetSteps;
  }

  void enable() {
    if (_isEnabled == true) return;
    digitalWriteFast(_enablePin, LOW);
    _isEnabled = true;
  }
  void disable() {
    if (_isEnabled == false) return;
    digitalWriteFast(_enablePin, HIGH);
    _isEnabled = false;
  }

  void runMovement() {
    int dirPinState = _isReversed ? -_direction : _direction;
    digitalWriteFast(_dirPin, dirPinState);

    // do one step
    digitalWriteFast(_stepPin, 1);
    delayMicroseconds(_pulseWidthUs);
    digitalWriteFast(_stepPin, 0);

    _steps += 1 * _direction;
  }

  void callback() {
    if (isAtTargetSteps()) { return _timer->pause(); }

    computeMovement();
    runMovement();

    _timer->setOverflow(_periodUs, MICROSEC_FORMAT);
    _timer->resume();
  };

  static void ISR_0() { steppers[0]->callback(); }
  static void ISR_1() { steppers[1]->callback(); }
};

// Define the static member variable
Stepper *Stepper::steppers[2] = {nullptr, nullptr};