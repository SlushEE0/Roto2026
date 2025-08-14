#ifndef __MOTOR_H
#define __MOTOR_H

#include <Arduino.h>
#include <FlexyStepper.h>
#include <config.h>

enum class StepperMoveType {
  ABSOLUTE_BLOCKING,
  RELATIVE_BLOCKING,
  ABSOLUTE_NONBLOCKING,
  RELATIVE_NONBLOCKING
};

class Motor {
private:
  FlexyStepper stepper;

  int pin_DIR;
  int pin_STEP;
  int pin_ENABLE;

  long target_steps;
  long position_steps;
  long accel = MOTOR_MAX_ACCEL * 0.5;
  long speed = MOTOR_MAX_SPEED * 0.7;

  bool isAlwaysEnabled;
  bool isEnabled;
  bool isReversed;

  long getMovePosition(long relativeSteps);

public:
  Motor();
  ~Motor();
  void connect(int dirPin, int stepPin, int enablePin, bool reversed = false);

  FlexyStepper *getStepper();

  void setSpeed(long newSpeed);
  void setAccel(long newAccel);
  void setReversed(bool isReversed);
  void setAlwaysEnabled(bool alwaysEnabled);
  void enable();
  void disable();

  long getPositionSteps();

  void moveAbsolute_blocking(long steps);
  void moveRelative_blocking(long steps);
  void moveAbsolute_nonblocking(long steps);
  void moveRelative_nonblocking(long steps);

  void
  constrainedMove(long steps, long accel, long speed, StepperMoveType moveType);

  void updateData();
  void
  processMovement(); // Call this regularly in main loop for non-blocking moves
  bool isMovementComplete(); // Check if non-blocking movement is finished
  void emergencyStop(); // Stop movement immediately
};

#endif // __MOTOR_H