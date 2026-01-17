#include "dead_reckoning.h"

#include <cmath>

namespace {
constexpr float kDegToRad = PI_F / 180.0f;
constexpr float kRadToDeg = 180.0f / PI_F;
constexpr float kDistanceTolCm = 1.0f; // 1 cm tolerance for distance
constexpr float kHeadingTolRad =
  2.0f * kDegToRad; // 2 degree tolerance for heading
} // namespace

DeadReckoningDrivetrain::DeadReckoningDrivetrain(Stepper& left,
                                                 Stepper& right,
                                                 BNO* imu)
    : _left(left), _right(right), _imu(imu), _queueHead(0), _queueTail(0),
      _queueCount(0), _isExecuting(false), _startLeftSteps(0),
      _startRightSteps(0), _startHeading(0.0f), _targetHeading(0.0f),
      _headingGain(2.0f), _turnGain(1.5f) {
}

void DeadReckoningDrivetrain::setIMU(BNO* imu) {
  _imu = imu;
}

void DeadReckoningDrivetrain::setHeadingGain(float gain) {
  _headingGain = gain;
}

void DeadReckoningDrivetrain::setTurnGain(float gain) {
  _turnGain = gain;
}

void DeadReckoningDrivetrain::stop() {
  _queueHead = 0;
  _queueTail = 0;
  _queueCount = 0;
  _isExecuting = false;
  _activeCommand.type = DeadReckoningCommandType::Idle;
  setWheelVelocities(0, 0);
}

bool DeadReckoningDrivetrain::isBusy() const {
  return _isExecuting || (_queueCount > 0);
}

float DeadReckoningDrivetrain::getHeading() const {
  if (_imu && _imu->isReady()) {
    return _imu->getRotation()->getYawRads();
  }
  return 0.0f;
}

void DeadReckoningDrivetrain::update() {
  // Process command queue
  if (!_isExecuting) {
    if (_queueCount > 0) {
      // Dequeue next command
      _activeCommand = _queue[_queueHead];
      _queueHead = (_queueHead + 1) % kDeadReckoningQueueSize;
      _queueCount--;
      _isExecuting = true;

      // Initialize command state
      _startLeftSteps = _left.currentPosition();
      _startRightSteps = _right.currentPosition();
      _startHeading = getHeading();

      if (_activeCommand.type == DeadReckoningCommandType::Turn) {
        _targetHeading =
          normalizeAngle(_startHeading + degToRad(_activeCommand.turn.degrees));
      } else if (_activeCommand.type ==
                 DeadReckoningCommandType::DriveStraight) {
        _targetHeading = _startHeading; // Maintain current heading
      }
    } else {
      // Idle - stop motors
      setWheelVelocities(0, 0);
      return;
    }
  }

  processCommand();
}

void DeadReckoningDrivetrain::processCommand() {
  switch (_activeCommand.type) {
    case DeadReckoningCommandType::DriveStraight:
      handleDriveStraight();
      break;
    case DeadReckoningCommandType::Turn:
      handleTurn();
      break;
    case DeadReckoningCommandType::Idle:
    default:
      _isExecuting = false;
      break;
  }
}

void DeadReckoningDrivetrain::handleDriveStraight() {
  // Calculate distance traveled using wheel encoders
  long currentLeft = _left.currentPosition();
  long currentRight = _right.currentPosition();

  float leftDist = cnv_stepsToCM(currentLeft - _startLeftSteps);
  float rightDist = cnv_stepsToCM(currentRight - _startRightSteps);
  float avgDist = (leftDist + rightDist) * 0.5f;

  float targetDist = fabsf(_activeCommand.straight.distanceCm);
  float distError = targetDist - fabsf(avgDist);

  // Check if we've reached the target distance
  if (distError <= kDistanceTolCm) {
    setWheelVelocities(0, 0);
    _isExecuting = false;
    return;
  }

  // Use IMU for heading correction
  float currentHeading = getHeading();
  float headingError = normalizeAngle(_targetHeading - currentHeading);
  float correction = headingError * _headingGain;

  // Base speed (negative if going backwards)
  float baseSpeed = _activeCommand.straight.speedCmPerSec;
  if (_activeCommand.straight.distanceCm < 0) {
    baseSpeed = -baseSpeed;
  }

  // Slow down near the end
  if (distError < 10.0f) {
    baseSpeed *= (distError / 10.0f);
    if (fabsf(baseSpeed) < 2.0f) {
      baseSpeed = (baseSpeed > 0) ? 2.0f : -2.0f;
    }
  }

  // Apply heading correction to wheel speeds
  float leftSpeed = baseSpeed - correction;
  float rightSpeed = baseSpeed + correction;

  setWheelVelocities(leftSpeed, rightSpeed);
}

void DeadReckoningDrivetrain::handleTurn() {
  // Use IMU only for turn control
  float currentHeading = getHeading();
  float error = normalizeAngle(_targetHeading - currentHeading);

  // Check if we've reached the target heading
  if (fabsf(error) <= kHeadingTolRad) {
    setWheelVelocities(0, 0);
    _isExecuting = false;
    return;
  }

  // P-control for turn speed
  float turnSpeed =
    error * _turnGain * kRadToDeg; // Convert to deg/s for scaling

  // Clamp to max speed
  float maxSpeed = fabsf(_activeCommand.turn.speedDegPerSec);
  if (turnSpeed > maxSpeed)
    turnSpeed = maxSpeed;
  if (turnSpeed < -maxSpeed)
    turnSpeed = -maxSpeed;

  // Minimum speed to overcome friction
  if (fabsf(turnSpeed) < 10.0f) {
    turnSpeed = (turnSpeed > 0) ? 10.0f : -10.0f;
  }

  // Convert deg/s to wheel speed (cm/s)
  // For a turn in place: wheelSpeed = omega * (trackWidth / 2)
  float wheelSpeed = degToRad(turnSpeed) * (DT_TRACK_WIDTH_CM / 2.0f);

  // Opposite wheel directions for turn in place
  setWheelVelocities(-wheelSpeed, wheelSpeed);
}

// Queueing functions
bool DeadReckoningDrivetrain::queueDriveStraight(float distanceCm,
                                                 float speedCmPerSec) {
  if (_queueCount >= kDeadReckoningQueueSize) {
    return false;
  }

  DeadReckoningCommand cmd;
  cmd.type = DeadReckoningCommandType::DriveStraight;
  cmd.straight.distanceCm = distanceCm;
  cmd.straight.speedCmPerSec = speedCmPerSec;

  _queue[_queueTail] = cmd;
  _queueTail = (_queueTail + 1) % kDeadReckoningQueueSize;
  _queueCount++;
  return true;
}

bool DeadReckoningDrivetrain::queueTurn(float degrees, float speedDegPerSec) {
  if (_queueCount >= kDeadReckoningQueueSize) {
    return false;
  }

  DeadReckoningCommand cmd;
  cmd.type = DeadReckoningCommandType::Turn;
  cmd.turn.degrees = degrees;
  cmd.turn.speedDegPerSec = speedDegPerSec;

  _queue[_queueTail] = cmd;
  _queueTail = (_queueTail + 1) % kDeadReckoningQueueSize;
  _queueCount++;
  return true;
}

// Helpers
void DeadReckoningDrivetrain::setWheelVelocities(float leftCmPerSec,
                                                 float rightCmPerSec) {
  float leftSteps = leftCmPerSec * StepsPerCM;
  float rightSteps = rightCmPerSec * StepsPerCM;
  _left.commandVelocity(leftSteps);
  _right.commandVelocity(rightSteps);
}

float DeadReckoningDrivetrain::normalizeAngle(float angle) {
  while (angle > PI_F)
    angle -= TWO_PI_F;
  while (angle < -PI_F)
    angle += TWO_PI_F;
  return angle;
}

float DeadReckoningDrivetrain::degToRad(float deg) {
  return deg * kDegToRad;
}

float DeadReckoningDrivetrain::radToDeg(float rad) {
  return rad * kRadToDeg;
}
