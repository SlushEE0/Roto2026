#include "drivetrain.h"

#include <cmath>

#if defined(__arm__) || defined(ARDUINO_ARCH_STM32)
#include <cmsis_gcc.h>
#endif

namespace {
constexpr float kDegToRad = PI_F / 180.0f;
constexpr float kRadToDeg = 180.0f / PI_F;
constexpr float kDistanceTolCm = 1.0f;
constexpr float kHeadingTolRad = 2.0f * kDegToRad;

#if defined(ARDUINO_ARCH_AVR)
bool interruptsAreEnabled() {
  return bitRead(SREG, SREG_I);
}
#elif defined(__arm__) || defined(ARDUINO_ARCH_STM32)
bool interruptsAreEnabled() {
  return (__get_PRIMASK() == 0U);
}
#else
bool interruptsAreEnabled() {
  return true;
}
#endif

class InterruptGuard {
public:
  InterruptGuard() : _wasEnabled(interruptsAreEnabled()) {
    if (_wasEnabled)
      noInterrupts();
  }
  ~InterruptGuard() {
    if (_wasEnabled)
      interrupts();
  }

private:
  bool _wasEnabled;
};
} // namespace

// ── Constructor ──────────────────────────────────────────────────────────────

DifferentialDrive::DifferentialDrive(Stepper& left, Stepper& right, BNO* imu)
    : _left(left), _right(right), _imu(imu), _queueHead(0), _queueTail(0),
      _queueCount(0), _isExecuting(false), _startLeftSteps(0),
      _startRightSteps(0), _startHeading(0.0f), _targetHeading(0.0f) {
  // Heading-hold PID (corrects drift while driving straight)
  _headingPID.setGains(2.0f, 0.0f, 0.05f);
  _headingPID.setOutputLimits(-90.0f, 90.0f);

  // Turn-in-place PID – gentle gains for smooth, controlled rotation
  _turnPID.setGains(1.2f, 0.05f, 0.15f);
  _turnPID.setOutputLimits(-120.0f, 120.0f);
}

// ── Configuration ────────────────────────────────────────────────────────────

void DifferentialDrive::setIMU(BNO* imu) {
  _imu = imu;
}

void DifferentialDrive::setHeadingPID(float kP, float kI, float kD) {
  _headingPID.setGains(kP, kI, kD);
}

void DifferentialDrive::setTurnPID(float kP, float kI, float kD) {
  _turnPID.setGains(kP, kI, kD);
}

// ── Control ──────────────────────────────────────────────────────────────────

void DifferentialDrive::stop() {
  InterruptGuard guard;
  _queueHead = 0;
  _queueTail = 0;
  _queueCount = 0;
  _isExecuting = false;
  _activeCommand.type = CommandType::Idle;
  setWheelVelocities(0, 0);
}

bool DifferentialDrive::isBusy() const {
  return _isExecuting || (_queueCount > 0);
}

float DifferentialDrive::getHeading() const {
  if (_imu && _imu->isReady())
    return _imu->getRotation()->getYawRads();
  return 0.0f;
}

// ── Main update loop ─────────────────────────────────────────────────────────

void DifferentialDrive::update() {
  if (!_isExecuting) {
    if (_queueCount > 0) {
      InterruptGuard guard;
      _activeCommand = _queue[_queueHead];
      _queueHead = (_queueHead + 1) % kDrivetrainQueueSize;
      _queueCount--;
      _isExecuting = true;

      _startLeftSteps = _left.currentPosition();
      _startRightSteps = _right.currentPosition();
      _startHeading = getHeading();

      if (_activeCommand.type == CommandType::Turn) {
        _targetHeading =
          normalizeAngle(_startHeading + degToRad(_activeCommand.angleDeg));
      } else {
        _targetHeading = _startHeading; // hold heading while driving
      }

      _headingPID.reset();
      _turnPID.reset();
    } else {
      setWheelVelocities(0, 0);
      return;
    }
  }

  processCommand();
}

void DifferentialDrive::processCommand() {
  switch (_activeCommand.type) {
    case CommandType::DriveStraight:
      handleDriveStraight();
      break;
    case CommandType::Turn:
      handleTurn();
      break;
    default:
      _isExecuting = false;
      break;
  }
}

// ── DriveStraight handler ────────────────────────────────────────────────────

void DifferentialDrive::handleDriveStraight() {
  // Distance traveled (average of both wheels)
  float leftDist = cnv_stepsToCM(_left.currentPosition() - _startLeftSteps);
  float rightDist = cnv_stepsToCM(_right.currentPosition() - _startRightSteps);
  float avgDist = (leftDist + rightDist) * 0.5f;

  float targetDist = fabsf(_activeCommand.distance);
  float remaining = targetDist - fabsf(avgDist);

  if (remaining <= kDistanceTolCm) {
    setWheelVelocities(0, 0);
    _isExecuting = false;
    return;
  }

  // Base speed — negative if driving backwards
  float baseSpeed = _activeCommand.linearSpeed;
  if (_activeCommand.distance < 0)
    baseSpeed = -baseSpeed;

  // Slow down in the last 10 cm
  if (remaining < 10.0f) {
    float scale = remaining / 10.0f;
    if (scale < 0.15f)
      scale = 0.15f;
    baseSpeed *= scale;
  }
  // Floor speed to avoid stalling
  if (fabsf(baseSpeed) < 2.0f)
    baseSpeed = copysignf(2.0f, baseSpeed);

  // No heading correction — drive both wheels at equal speed.
  // The matched stepper acceleration ramps keep it straight enough.
  setWheelVelocities(baseSpeed, baseSpeed);
}

// ── Turn handler ─────────────────────────────────────────────────────────────
// Pure open-loop: compute how many steps each wheel needs for the desired
// angle using geometry, then let the stepper's built-in position mode
// (with acceleration ramp) do the rest.  No gyro/IMU involved.
//
//   arc = angle_rad * (TRACK_WIDTH / 2)
//   steps = arc * StepsPerCM
//
// For an in-place turn the wheels spin in opposite directions.
// Positive angleDeg → CCW → left wheel backward, right wheel forward.

void DifferentialDrive::handleTurn() {
  // On the first call after the command is dequeued, _startLeftSteps /
  // _startRightSteps are already latched by update().  We use the stepper's
  // own isBusy() to know when the move is done.

  static bool turnCommandSent = false;

  if (!turnCommandSent) {
    float angleRad = degToRad(_activeCommand.angleDeg); // signed
    float arcCm = angleRad * (DT_TRACK_WIDTH_CM / 2.0f);
    long arcSteps = cnv_CMToSteps(arcCm);

    // Turn speed: convert deg/s → wheel cm/s → steps/s
    float wheelCmPerSec =
      degToRad(fabsf(_activeCommand.turnSpeed)) * (DT_TRACK_WIDTH_CM / 2.0f);
    float wheelStepsPerSec = wheelCmPerSec * StepsPerCM;

    // Left wheel goes –arcSteps, right wheel goes +arcSteps
    _left.commandRelative(-arcSteps, wheelStepsPerSec);
    _right.commandRelative(arcSteps, wheelStepsPerSec);

    turnCommandSent = true;
    return;
  }

  // Wait for both wheels to finish their moves
  if (!_left.isBusy() && !_right.isBusy()) {
    turnCommandSent = false;
    _isExecuting = false;
  }
}

// ── Queue: with explicit speeds ──────────────────────────────────────────────

bool DifferentialDrive::queueDriveStraight(float distanceCm,
                                           float speedCmPerSec) {
  InterruptGuard guard;
  if (_queueCount >= kDrivetrainQueueSize)
    return false;

  DriveCommand cmd;
  cmd.type = CommandType::DriveStraight;
  cmd.distance = distanceCm;
  cmd.linearSpeed = fabsf(speedCmPerSec);

  _queue[_queueTail] = cmd;
  _queueTail = (_queueTail + 1) % kDrivetrainQueueSize;
  _queueCount++;
  return true;
}

bool DifferentialDrive::queueTurnDegrees(float deg, float speedDegPerSec) {
  InterruptGuard guard;
  if (_queueCount >= kDrivetrainQueueSize)
    return false;

  DriveCommand cmd;
  cmd.type = CommandType::Turn;
  cmd.angleDeg = deg;
  cmd.turnSpeed = fabsf(speedDegPerSec);

  _queue[_queueTail] = cmd;
  _queueTail = (_queueTail + 1) % kDrivetrainQueueSize;
  _queueCount++;
  return true;
}

// ── Queue: speed-less (for setTimeTarget) ────────────────────────────────────

bool DifferentialDrive::queueDrive(float distanceCm) {
  return queueDriveStraight(distanceCm, 0.0f);
}

bool DifferentialDrive::queueTurn(float deg) {
  return queueTurnDegrees(deg, 0.0f);
}

// ── setTimeTarget ────────────────────────────────────────────────────────────

// Helper: estimate time for a trapezoidal motion profile.
// Given distance (in consistent units) and cruise speed, account for
// acceleration ramp-up and ramp-down.  If the move is too short to
// reach cruise speed, it's a triangular profile.
static float trapezoidalTime(float distance, float cruiseSpeed, float accel) {
  distance = fabsf(distance);
  cruiseSpeed = fabsf(cruiseSpeed);
  if (cruiseSpeed < 1e-3f)
    return 0.0f;
  if (accel < 1e-3f)
    return distance / cruiseSpeed; // no accel, instant

  // Time and distance to ramp up to cruise speed
  float tRamp = cruiseSpeed / accel;
  float dRamp = 0.5f * accel * tRamp * tRamp; // distance for one ramp

  if (2.0f * dRamp >= distance) {
    // Triangular profile — never reaches cruise speed
    // d = 2 * (1/2 * a * t_half^2)  =>  t_half = sqrt(d / (2*a))
    // but we accelerate then decelerate symmetrically, so from the
    // full distance: t_total = 2 * sqrt(distance / (2 * accel))
    //               = 2 * sqrt(d/a) / sqrt(2)
    //               = sqrt(2 * distance / accel)
    return sqrtf(2.0f * distance / accel);
  }

  // Trapezoidal: ramp-up + cruise + ramp-down
  float dCruise = distance - 2.0f * dRamp;
  float tCruise = dCruise / cruiseSpeed;
  return 2.0f * tRamp + tCruise;
}

bool DifferentialDrive::setTimeTarget(float totalSeconds) {
  if (totalSeconds <= 0.0f || _queueCount == 0)
    return false;

  // Stepper acceleration in cm/s² (for linear moves) and the raw steps/s²
  float accelSteps = (float)MOTOR_MAX_ACCEL;
  float accelCmS2 = accelSteps / StepsPerCM;

  // 1) Count unresolved totals & subtract time consumed by explicit-speed cmds.
  //    For turns, convert angle to wheel arc distance (cm) so everything
  //    is in the same unit for time budgeting.
  float totalLinearDist = 0.0f; // cm  (unresolved drive segments)
  float totalTurnArcCm = 0.0f;  // cm  (unresolved turn arcs)
  float usedTime = 0.0f;        // seconds consumed by explicit-speed cmds
  std::size_t numUnresolvedDrive = 0;
  std::size_t numUnresolvedTurn = 0;

  for (std::size_t i = 0; i < _queueCount; ++i) {
    std::size_t idx = (_queueHead + i) % kDrivetrainQueueSize;
    const DriveCommand& cmd = _queue[idx];

    if (cmd.type == CommandType::DriveStraight) {
      if (cmd.linearSpeed == 0.0f) {
        totalLinearDist += fabsf(cmd.distance);
        numUnresolvedDrive++;
      } else {
        // Estimate with trapezoidal profile
        usedTime += trapezoidalTime(cmd.distance, cmd.linearSpeed, accelCmS2);
      }
    } else if (cmd.type == CommandType::Turn) {
      // Convert degrees → wheel arc cm
      float arcCm = fabsf(degToRad(cmd.angleDeg)) * (DT_TRACK_WIDTH_CM / 2.0f);
      if (cmd.turnSpeed == 0.0f) {
        totalTurnArcCm += arcCm;
        numUnresolvedTurn++;
      } else {
        // turnSpeed is deg/s → convert to wheel cm/s for time estimate
        float wheelCmS =
          degToRad(fabsf(cmd.turnSpeed)) * (DT_TRACK_WIDTH_CM / 2.0f);
        usedTime += trapezoidalTime(arcCm, wheelCmS, accelCmS2);
      }
    }
  }

  float remaining = totalSeconds - usedTime;
  if (remaining < 0.1f)
    remaining = 0.1f; // safety floor

  // Nothing to resolve?
  float totalUnresolvedCm = totalLinearDist + totalTurnArcCm;
  if (totalUnresolvedCm < 1e-3f)
    return true;

  // 2) Distribute remaining time proportionally by distance.
  //    Both drive and turn distances are already in cm.
  float timeDrive = remaining;
  float timeTurn = remaining;
  if (totalUnresolvedCm > 1e-3f) {
    timeDrive = remaining * (totalLinearDist / totalUnresolvedCm);
    timeTurn = remaining * (totalTurnArcCm / totalUnresolvedCm);
  }
  if (timeDrive < 0.05f)
    timeDrive = 0.05f;
  if (timeTurn < 0.05f)
    timeTurn = 0.05f;

  // 3) Iteratively solve for cruise speed that, with trapezoidal profiles,
  //    fills the time budget.  We need to account for acceleration ramps
  //    which make the real time > distance/speed.
  //
  //    Start with the simple distance/time estimate, then nudge up since
  //    the ramps mean we need a higher cruise speed to finish on time.
  //    A few Newton-ish iterations converge quickly.

  // --- Linear speed ---
  float linearSpeed = 5.0f;
  if (totalLinearDist > 1e-3f && numUnresolvedDrive > 0) {
    linearSpeed = totalLinearDist / timeDrive; // initial guess
    // Avg distance per segment for ramp estimation
    float avgDist = totalLinearDist / (float)numUnresolvedDrive;
    for (int iter = 0; iter < 5; ++iter) {
      float estTime = (float)numUnresolvedDrive *
                      trapezoidalTime(avgDist, linearSpeed, accelCmS2);
      float ratio = estTime / timeDrive;
      if (ratio < 1e-3f)
        break;
      linearSpeed *= ratio; // scale up to fill the budget
    }
  }

  // --- Turn speed (in deg/s) ---
  float turnSpeed = 30.0f;
  if (totalTurnArcCm > 1e-3f && numUnresolvedTurn > 0) {
    // Work in wheel-arc cm/s, then convert back to deg/s
    float wheelCmS = totalTurnArcCm / timeTurn; // initial guess
    float avgArc = totalTurnArcCm / (float)numUnresolvedTurn;
    for (int iter = 0; iter < 5; ++iter) {
      float estTime =
        (float)numUnresolvedTurn * trapezoidalTime(avgArc, wheelCmS, accelCmS2);
      float ratio = estTime / timeTurn;
      if (ratio < 1e-3f)
        break;
      wheelCmS *= ratio;
    }
    // Convert wheel cm/s back to deg/s
    turnSpeed = radToDeg(wheelCmS / (DT_TRACK_WIDTH_CM / 2.0f));
  }

  // Clamp to hardware limits
  float maxLinear = (float)MOTOR_MAX_SPEED / StepsPerCM;
  constexpr float kMaxTurnSpeedDegS = 90.0f;
  if (linearSpeed > maxLinear)
    linearSpeed = maxLinear;
  if (turnSpeed > kMaxTurnSpeedDegS)
    turnSpeed = kMaxTurnSpeedDegS;

  // 4) Assign speeds to every unresolved command.
  for (std::size_t i = 0; i < _queueCount; ++i) {
    std::size_t idx = (_queueHead + i) % kDrivetrainQueueSize;
    DriveCommand& cmd = _queue[idx];

    if (cmd.type == CommandType::DriveStraight && cmd.linearSpeed == 0.0f)
      cmd.linearSpeed = linearSpeed;
    else if (cmd.type == CommandType::Turn && cmd.turnSpeed == 0.0f)
      cmd.turnSpeed = turnSpeed;
  }

  return true;
}

// ── Helpers ──────────────────────────────────────────────────────────────────

void DifferentialDrive::setWheelVelocities(float leftCmPerSec,
                                           float rightCmPerSec) {
  _left.commandVelocity(leftCmPerSec * StepsPerCM);
  _right.commandVelocity(rightCmPerSec * StepsPerCM);
}

float DifferentialDrive::normalizeAngle(float angle) const {
  while (angle > PI_F)
    angle -= TWO_PI_F;
  while (angle < -PI_F)
    angle += TWO_PI_F;
  return angle;
}

float DifferentialDrive::degToRad(float deg) const {
  return deg * kDegToRad;
}

float DifferentialDrive::radToDeg(float rad) const {
  return rad * kRadToDeg;
}
