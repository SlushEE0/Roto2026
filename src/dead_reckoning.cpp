#include "dead_reckoning.h"

#include <Arduino.h>
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
  float currentHeading = getHeading();
  float error = normalizeAngle(_targetHeading - currentHeading);

  // Check if we've reached the target heading
  if (fabsf(error) <= kHeadingTolRad) {
    setWheelVelocities(0, 0);
    _isExecuting = false;
    if (_useTurnPID) _turnPID.reset();
    return;
  }

  float turnSpeed;
  if (_useTurnPID) {
    // PID control path
    uint32_t now = millis();
    float dt = (_lastTurnUpdateMs == 0) ? 0.01f
               : (float)(now - _lastTurnUpdateMs) * 0.001f;
    _lastTurnUpdateMs = now;
    if (dt <= 0.0f || dt > 0.5f) dt = 0.01f; // sanity clamp

    turnSpeed = _turnPID.compute(error * kRadToDeg, dt); // error in degrees
  } else {
    // Legacy P-only path
    turnSpeed = error * _turnGain * kRadToDeg;
  }

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

void DeadReckoningDrivetrain::setTurnPID(float kP, float kI, float kD) {
  _turnPID.setGains(kP, kI, kD);
  _turnPID.setOutputLimits(-360.0f, 360.0f); // deg/s
  _turnPID.reset();
  _useTurnPID = true;
  _lastTurnUpdateMs = 0;
}

// ---------------------------------------------------------------------------
// Ziegler–Nichols Turn Tuner
// ---------------------------------------------------------------------------
// Procedure (classic Z-N "ultimate gain" method):
//   1.  Start with a low P-only gain and command a fixed-angle turn.
//   2.  Increase Kp until the heading response shows sustained oscillation
//       (the robot overshoots and oscillates around the target).
//   3.  Record the ultimate gain Ku and the oscillation period Tu.
//   4.  Compute PID gains from the Z-N table:
//         Kp = 0.6  * Ku
//         Ki = 1.2  * Ku / Tu
//         Kd = 0.075 * Ku * Tu
//   5.  Apply them to _turnPID.
//
// This method BLOCKS – call it once from setup() before queueing moves.
// Watch Serial1 for real-time output.
// ---------------------------------------------------------------------------
void DeadReckoningDrivetrain::runZieglerNicholsTurnTune(
    float targetDeg,
    float maxSpeedDeg,
    float startKp,
    float kpStep,
    float maxKp) {

  if (!_imu || !_imu->isReady()) {
    Serial1.println("[ZN] ERROR: IMU not ready, cannot tune");
    return;
  }

  Serial1.println("[ZN] ======== Ziegler-Nichols Turn Tuner ========");
  Serial1.print("[ZN] Target oscillation: +/- ");
  Serial1.print(targetDeg, 1);
  Serial1.println(" deg");
  Serial1.print("[ZN] Kp range: ");
  Serial1.print(startKp, 2);
  Serial1.print(" -> ");
  Serial1.print(maxKp, 2);
  Serial1.print(", step ");
  Serial1.println(kpStep, 2);
  Serial1.println("[ZN] Starting in 2 seconds...");
  delay(2000);

  const float targetRad = targetDeg * kDegToRad;
  const float kSettleTimeSec = 4.0f;    // how long to run each Kp trial
  const float kSampleIntervalMs = 10.0f;

  float Ku = 0.0f;
  float Tu = 0.0f;

  for (float kp = startKp; kp <= maxKp; kp += kpStep) {
    Serial1.print("[ZN] Testing Kp = ");
    Serial1.println(kp, 3);

    // Tare heading to zero
    _imu->update();
    float baseHeading = _imu->getRotation()->getYawRads();
    float target = normalizeAngle(baseHeading + targetRad);

    // --- Run P-only control for kSettleTimeSec and record heading ---
    // We track zero-crossings of the error to detect oscillation.
    const int kMaxSamples = (int)(kSettleTimeSec * 1000.0f / kSampleIntervalMs);
    int zeroCrossings = 0;
    float prevError = 0.0f;
    bool firstSample = true;

    // Timestamps of zero-crossings for period measurement
    static constexpr int kMaxCrossings = 32;
    uint32_t crossTimes[kMaxCrossings];
    int crossCount = 0;

    float peakError = 0.0f;

    uint32_t trialStart = millis();

    for (int s = 0; s < kMaxSamples; ++s) {
      _imu->update();
      float heading = _imu->getRotation()->getYawRads();
      float error = normalizeAngle(target - heading);

      // Track peak error magnitude
      if (fabsf(error) > peakError) peakError = fabsf(error);

      // Detect sign change (zero crossing)
      if (!firstSample && prevError * error < 0.0f) {
        zeroCrossings++;
        if (crossCount < kMaxCrossings) {
          crossTimes[crossCount++] = millis();
        }
      }
      firstSample = false;
      prevError = error;

      // P-only control
      float turnSpeed = error * kp * kRadToDeg; // deg/s
      if (turnSpeed > maxSpeedDeg) turnSpeed = maxSpeedDeg;
      if (turnSpeed < -maxSpeedDeg) turnSpeed = -maxSpeedDeg;
      if (fabsf(turnSpeed) < 5.0f) {
        turnSpeed = (turnSpeed > 0) ? 5.0f : -5.0f;
      }
      float wheelSpeed = degToRad(turnSpeed) * (DT_TRACK_WIDTH_CM / 2.0f);
      setWheelVelocities(-wheelSpeed, wheelSpeed);

      delay((int)kSampleIntervalMs);
    }

    // Stop motors
    setWheelVelocities(0, 0);

    Serial1.print("[ZN]   Zero crossings: ");
    Serial1.print(zeroCrossings);
    Serial1.print("  Peak error: ");
    Serial1.print(peakError * kRadToDeg, 1);
    Serial1.println(" deg");

    // Sustained oscillation = at least 4 zero crossings
    // (2 full cycles) and error never decayed to near-zero
    if (zeroCrossings >= 4 && peakError > (targetRad * 0.15f)) {
      Ku = kp;

      // Compute average oscillation period from zero-crossing timestamps
      if (crossCount >= 4) {
        // Each full cycle = 2 zero crossings
        float totalTime = (float)(crossTimes[crossCount - 1] - crossTimes[0]);
        float fullCycles = (float)(crossCount - 1) / 2.0f;
        Tu = (totalTime / fullCycles) / 1000.0f; // seconds
      }

      Serial1.print("[ZN] >>> Sustained oscillation detected at Ku = ");
      Serial1.print(Ku, 3);
      Serial1.print(", Tu = ");
      Serial1.print(Tu, 4);
      Serial1.println(" s");
      break;
    }

    // Let the robot settle before next trial
    Serial1.println("[ZN]   No sustained oscillation, increasing Kp...");
    delay(1500);
  }

  if (Ku < kpStep * 0.5f || Tu < 0.01f) {
    Serial1.println("[ZN] FAILED: Could not find sustained oscillation.");
    Serial1.println("[ZN] Try increasing maxKp or reducing kpStep.");
    setWheelVelocities(0, 0);
    return;
  }

  // --- Compute Z-N PID gains ---
  float znKp = 0.6f  * Ku;
  float znKi = 1.2f  * Ku / Tu;
  float znKd = 0.075f * Ku * Tu;

  Serial1.println("[ZN] -------- Ziegler-Nichols Results --------");
  Serial1.print("[ZN]   Ku = "); Serial1.println(Ku, 4);
  Serial1.print("[ZN]   Tu = "); Serial1.print(Tu, 4); Serial1.println(" s");
  Serial1.println("[ZN]   --- PID Gains ---");
  Serial1.print("[ZN]   Kp = "); Serial1.println(znKp, 4);
  Serial1.print("[ZN]   Ki = "); Serial1.println(znKi, 4);
  Serial1.print("[ZN]   Kd = "); Serial1.println(znKd, 4);
  Serial1.println("[ZN] ------- P-only / PI alternatives -------");
  Serial1.print("[ZN]   P-only  Kp = "); Serial1.println(0.5f * Ku, 4);
  Serial1.print("[ZN]   PI  Kp = "); Serial1.print(0.45f * Ku, 4);
  Serial1.print(", Ki = "); Serial1.println(0.54f * Ku / Tu, 4);
  Serial1.println("[ZN] ------------------------------------------");

  // Apply the PID gains
  setTurnPID(znKp, znKi, znKd);
  Serial1.println("[ZN] PID gains applied to turn controller.");
  Serial1.println("[ZN] ======== Tuning Complete ========");
}
