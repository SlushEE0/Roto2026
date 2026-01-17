#include "drivetrain.h"

#include <cmath>

#if defined(__arm__) || defined(ARDUINO_ARCH_STM32)
#include <cmsis_gcc.h>
#endif

namespace {
constexpr float kDegToRad          = PI_F / 180.0f;
constexpr float kRadToDeg          = 180.0f / PI_F;
constexpr float kVelocityEpsilon   = 1e-3f;
constexpr float kPoseDistanceTolCm = 1.0f;
constexpr float kPoseHeadingTolRad = 2.0f * kDegToRad;

#if defined(ARDUINO_ARCH_AVR)
bool interruptsAreEnabled() { return bitRead(SREG, SREG_I); }
#elif defined(__arm__) || defined(ARDUINO_ARCH_STM32)
bool interruptsAreEnabled() { return (__get_PRIMASK() == 0U); }
#else
bool interruptsAreEnabled() { return true; }
#endif

class InterruptGuard {
    public:
  InterruptGuard() : _wasEnabled(interruptsAreEnabled()) {
    if (_wasEnabled) { noInterrupts(); }
  }
  ~InterruptGuard() {
    if (_wasEnabled) { interrupts(); }
  }

    private:
  bool _wasEnabled;
};
} // namespace

DifferentialDrive::DifferentialDrive(Stepper  &left,
                                     Stepper  &right,
                                     Odometry *filter,
                                     BNO      *imu)
  : _left(left),
    _right(right),
    _filter(filter),
    _imu(imu),
    _queueHead(0),
    _queueTail(0),
    _queueCount(0),
    _isExecuting(false),
    _startLeftSteps(0),
    _startRightSteps(0),
    _targetHeading(0.0f),
    _subState(SubState::Init) {
  // Default PID gains - tune these for your robot
  _linearPID.setGains(2.0f, 0.1f, 0.05f);   // Distance PID
  _linearPID.setOutputLimits(-100.0f, 100.0f);  // cm/s
  _linearPID.setIntegratorLimits(-50.0f, 50.0f);
  
  _angularPID.setGains(3.0f, 0.2f, 0.1f);   // Heading PID
  _angularPID.setOutputLimits(-180.0f, 180.0f);  // deg/s
  _angularPID.setIntegratorLimits(-90.0f, 90.0f);
  
  if (_filter) { _filter->reset(); }
}

void DifferentialDrive::setFilter(Odometry *filter) {
  _filter = filter;
  if (_filter) { _filter->reset(getPose()); }
}

void DifferentialDrive::setIMU(BNO *imu) { _imu = imu; }

void DifferentialDrive::setLinearPID(float kP, float kI, float kD) {
  _linearPID.setGains(kP, kI, kD);
}

void DifferentialDrive::setAngularPID(float kP, float kI, float kD) {
  _angularPID.setGains(kP, kI, kD);
}

void DifferentialDrive::resetPose(const Pose &pose) {
  if (_filter) { _filter->reset(pose); }
  // If no filter, we might want to store an offset, but for now assume filter is primary source
}

void DifferentialDrive::stop() {
  InterruptGuard guard;
  _queueHead   = 0;
  _queueTail   = 0;
  _queueCount  = 0;
  _isExecuting = false;
  _activeCommand.type = DrivetrainCommandType::Idle;
  setWheelVelocities(0, 0);
}

bool DifferentialDrive::isBusy() const {
  return _isExecuting || (_queueCount > 0);
}

Pose DifferentialDrive::getPose() const {
  if (_filter) { return _filter->getPose(); }
  return Pose(); // Return zero pose if no filter
}

float DifferentialDrive::getHeading() const {
  if (_filter) { return _filter->getTheta(); }
  if (_imu) { return _imu->getRotation()->getYawRads(); }
  return 0.0f;
}

void DifferentialDrive::update() {
  // =========================================================================
  // 1. COMPUTE ODOMETRY FROM WHEEL ENCODERS (stepper positions)
  // =========================================================================
  long leftSteps  = _left.currentPosition();
  long rightSteps = _right.currentPosition();

  static long prevLeft = 0;
  static long prevRight = 0;
  static unsigned long prevMicros = 0;
  
  unsigned long now = micros();
  float dt = (float)(now - prevMicros) * 1e-6f;
  prevMicros = now;
  
  // Compute delta steps since last update
  long deltaLeft  = leftSteps - prevLeft;
  long deltaRight = rightSteps - prevRight;
  prevLeft  = leftSteps;
  prevRight = rightSteps;
  
  if (_filter && dt > 1e-6f) {
    // =========================================================================
    // DIFFERENTIAL DRIVE KINEMATICS
    // =========================================================================
    // Convert step deltas to distances (cm)
    float dlCm = cnv_stepsToCM(deltaLeft);
    float drCm = cnv_stepsToCM(deltaRight);
    
    // Compute linear velocity (v) and angular velocity (omega)
    // v = (v_left + v_right) / 2 = (dl + dr) / (2 * dt)
    // omega = (v_right - v_left) / track_width = (dr - dl) / (W * dt)
    float dCenterCm = (dlCm + drCm) * 0.5f;
    float dThetaRad = (drCm - dlCm) / DT_TRACK_WIDTH_CM;
    
    // Convert to velocities
    float v     = dCenterCm / dt;  // cm/s
    float omega = dThetaRad / dt;  // rad/s
    
    // =========================================================================
    // EKF PREDICTION STEP (with slip detection if IMU available)
    // =========================================================================
    if (_imu && _imu->isReady()) {
      // Get IMU gyro Z (yaw rate) for slip detection
      GyroData gyro = _imu->getGyroData();
      float imuOmega = gyro.z;  // rad/s (gyro Z axis = yaw rate)
      
      // Use slip-aware prediction that compares wheel vs IMU omega
      _filter->predictWithSlipDetection(v, omega, imuOmega, dt);
      
      // EKF CORRECTION STEP (with IMU yaw)
      Rotation* rot = _imu->getRotation();
      if (rot) {
        _filter->correct(rot->getYawRads());
      }
    } else {
      // No IMU available - use standard prediction (no slip detection)
      _filter->predict(v, omega, dt);
    }
  }

  // =========================================================================
  // 2. PROCESS COMMAND QUEUE
  // =========================================================================
  if (!_isExecuting) {
    if (_queueCount > 0) {
      // Dequeue
      InterruptGuard guard;
      _activeCommand = _queue[_queueHead];
      _queueHead     = (_queueHead + 1) % kDrivetrainQueueSize;
      _queueCount--;
      _isExecuting = true;
      
      // Initialize command state
      _startPose = getPose();
      _startLeftSteps = _left.currentPosition();
      _startRightSteps = _right.currentPosition();
      _subState = SubState::Init;
      
      if (_activeCommand.type == DrivetrainCommandType::TurnDegrees) {
          _targetHeading = normalizeAngle(getHeading() + degToRad(_activeCommand.turn.degrees));
      } else if (_activeCommand.type == DrivetrainCommandType::FollowTrajectory) {
          _activeCommand.trajectory.currentIndex = 0;
      }
    } else {
      setWheelVelocities(0, 0); // Idle
      return;
    }
  }

  processCommand();
}

void DifferentialDrive::processCommand() {
  switch (_activeCommand.type) {
    case DrivetrainCommandType::TurnDegrees:
      handleTurnDegrees();
      break;
    case DrivetrainCommandType::MoveToPose:
      handleMoveToPose();
      break;
    case DrivetrainCommandType::FollowTrajectory:
      handleFollowTrajectory();
      break;
    case DrivetrainCommandType::Idle:
    default:
      _isExecuting = false;
      break;
  }
}

void DifferentialDrive::handleTurnDegrees() {
  float currentHeading = getHeading();
  float error = normalizeAngle(_targetHeading - currentHeading);

  if (fabsf(error) <= kPoseHeadingTolRad) {
    setWheelVelocities(0, 0);
    _angularPID.reset();
    _isExecuting = false;
    return;
  }

  // Use angular PID for turn control
  // dt is approximate since we don't track it per-command; use a nominal value
  float dt = 0.012f;  // ~12ms update rate
  float turnSpeedDegSec = _angularPID.compute(error * kRadToDeg, dt);
  
  // Clamp to commanded max speed
  float maxSpeed = fabsf(_activeCommand.turn.speedDegPerSec);
  if (turnSpeedDegSec > maxSpeed) turnSpeedDegSec = maxSpeed;
  if (turnSpeedDegSec < -maxSpeed) turnSpeedDegSec = -maxSpeed;
  
  // Minimum speed to overcome friction
  if (fabsf(turnSpeedDegSec) < 10.0f && fabsf(error) > kPoseHeadingTolRad) {
    turnSpeedDegSec = (turnSpeedDegSec > 0 ? 10.0f : -10.0f);
  }

  // Convert deg/s to cm/s for wheels
  float wheelSpeed = degToRad(turnSpeedDegSec) * (DT_TRACK_WIDTH_CM / 2.0f);

  setWheelVelocities(-wheelSpeed, wheelSpeed);
}

void DifferentialDrive::handleMoveToPose() {
    Pose currentPose = getPose();
    // Reconstruct target pose from stored floats
    Rotation targetRot;
    targetRot.setYawRads(_activeCommand.pose.targetYaw);
    Pose targetPose(_activeCommand.pose.targetX, _activeCommand.pose.targetY, targetRot);
    
    float dx = targetPose.x - currentPose.x;
    float dy = targetPose.y - currentPose.y;
    float dist = sqrtf(dx*dx + dy*dy);
    float targetHeading = atan2f(dy, dx);
    
    // Nominal dt for PID (update rate ~12ms)
    // PID controller uses seconds
    static float prevTime = 0.0f;
    float now = millis() * 1e-3f;
    float dt = now - prevTime;
    prevTime = now;

    switch (_subState) {
        case SubState::Init:
            _linearPID.reset();
            _angularPID.reset();
            _subState = SubState::AlignToTarget;
            break;
            
        case SubState::AlignToTarget: {
            float headingError = normalizeAngle(targetHeading - currentPose.rot.getYawRads());
            if (fabsf(headingError) < kPoseHeadingTolRad) {
                _subState = SubState::DriveToTarget;
                _linearPID.reset();
                _angularPID.reset();
                _startLeftSteps = _left.currentPosition();
                _startRightSteps = _right.currentPosition();
            } else {
                // Use angular PID for turning
                float turnSpeedDegSec = _angularPID.compute(headingError * kRadToDeg, dt);
                float maxSpeed = _activeCommand.pose.turnSpeed;
                if (turnSpeedDegSec > maxSpeed) turnSpeedDegSec = maxSpeed;
                if (turnSpeedDegSec < -maxSpeed) turnSpeedDegSec = -maxSpeed;
                if (fabsf(turnSpeedDegSec) < 10.0f) turnSpeedDegSec = (turnSpeedDegSec > 0 ? 10.0f : -10.0f);
                float wheelSpeed = degToRad(turnSpeedDegSec) * (DT_TRACK_WIDTH_CM / 2.0f);
                setWheelVelocities(-wheelSpeed, wheelSpeed);
            }
            break;
        }
            
        case SubState::DriveToTarget: {
            if (dist < kPoseDistanceTolCm) {
                _subState = SubState::FinalAlign;
                _angularPID.reset();
            } else {
                // Re-calculate target heading dynamically to drive toward point
                float dynamicTargetHeading = atan2f(dy, dx);
                float headingError = normalizeAngle(dynamicTargetHeading - currentPose.rot.getYawRads());
                
                // If we deviate too much, stop and realign
                if (fabsf(headingError) > degToRad(30.0f)) {
                    _subState = SubState::AlignToTarget;
                    _angularPID.reset();
                    return;
                }
                
                // Use linear PID for distance control
                float linearSpeed = _linearPID.compute(dist, dt);
                
                // Clamp to commanded max speed
                float maxLinear = _activeCommand.pose.linearSpeed;
                if (linearSpeed > maxLinear) linearSpeed = maxLinear;
                if (linearSpeed < 5.0f) linearSpeed = 5.0f;  // Minimum speed
                
                // Use angular PID for heading correction while driving
                float correction = _angularPID.compute(headingError * kRadToDeg, dt) * 0.1f;  // Scale down for correction
                
                setWheelVelocities(linearSpeed - correction, linearSpeed + correction);
            }
            break;
        }
            
        case SubState::FinalAlign: {
            float finalHeading = _activeCommand.pose.targetYaw;
            float headingError = normalizeAngle(finalHeading - currentPose.rot.getYawRads());
            if (fabsf(headingError) < kPoseHeadingTolRad) {
                _subState = SubState::Done;
                setWheelVelocities(0, 0);
                _isExecuting = false;
            } else {
                // Use angular PID for final alignment
                float turnSpeedDegSec = _angularPID.compute(headingError * kRadToDeg, dt);
                float maxSpeed = _activeCommand.pose.turnSpeed;
                if (turnSpeedDegSec > maxSpeed) turnSpeedDegSec = maxSpeed;
                if (turnSpeedDegSec < -maxSpeed) turnSpeedDegSec = -maxSpeed;
                if (fabsf(turnSpeedDegSec) < 10.0f) turnSpeedDegSec = (turnSpeedDegSec > 0 ? 10.0f : -10.0f);
                float wheelSpeed = degToRad(turnSpeedDegSec) * (DT_TRACK_WIDTH_CM / 2.0f);
                setWheelVelocities(-wheelSpeed, wheelSpeed);
            }
            break;
        }
        
        case SubState::Done:
            _isExecuting = false;
            break;
    }
}

void DifferentialDrive::handleFollowTrajectory() {
    auto& trajData = _activeCommand.trajectory;
    
    if (trajData.currentIndex >= trajData.count) {
        _isExecuting = false;
        setWheelVelocities(0, 0);
        return;
    }
    
    Pose targetPose = trajData.points[trajData.currentIndex];
    Pose currentPose = getPose();
    float dx = targetPose.x - currentPose.x;
    float dy = targetPose.y - currentPose.y;
    float dist = sqrtf(dx*dx + dy*dy);
    
    // Nominal dt for PID
    float dt = 0.012f;
    
    float targetHeading = atan2f(dy, dx);
    float headingError = normalizeAngle(targetHeading - currentPose.rot.getYawRads());
    
    if (dist < kPoseDistanceTolCm) {
        // Reached point, move to next
        trajData.currentIndex++;
        _linearPID.reset();
        _angularPID.reset();
        return;
    }
    
    // If heading error is large, turn in place
    if (fabsf(headingError) > degToRad(20.0f)) {
        // Use angular PID for turning
        float turnSpeedDegSec = _angularPID.compute(headingError * kRadToDeg, dt);
        float maxSpeed = trajData.turnSpeed;
        if (turnSpeedDegSec > maxSpeed) turnSpeedDegSec = maxSpeed;
        if (turnSpeedDegSec < -maxSpeed) turnSpeedDegSec = -maxSpeed;
        float wheelSpeed = degToRad(turnSpeedDegSec) * (DT_TRACK_WIDTH_CM / 2.0f);
        setWheelVelocities(-wheelSpeed, wheelSpeed);
    } else {
        // Drive and correct using PID
        // Linear PID on distance
        float linearSpeed = _linearPID.compute(dist, dt);
        float maxLinear = trajData.linearSpeed;
        if (linearSpeed > maxLinear) linearSpeed = maxLinear;
        if (linearSpeed < 5.0f) linearSpeed = 5.0f;
        
        // Angular PID for heading correction (scaled down for while-driving correction)
        float correction = _angularPID.compute(headingError * kRadToDeg, dt) * 0.1f;
        
        // Slow down if turning sharp
        linearSpeed *= cosf(headingError);
        
        setWheelVelocities(linearSpeed - correction, linearSpeed + correction);
    }
}

// Queueing functions
bool DifferentialDrive::queueTurnDegrees(float degrees, float speedDegPerSec) {
  InterruptGuard guard;
  if (_queueCount >= kDrivetrainQueueSize) return false;
  
  DrivetrainCommand cmd;
  cmd.type = DrivetrainCommandType::TurnDegrees;
  cmd.turn.degrees = degrees;
  cmd.turn.speedDegPerSec = speedDegPerSec;
  
  _queue[_queueTail] = cmd;
  _queueTail = (_queueTail + 1) % kDrivetrainQueueSize;
  _queueCount++;
  return true;
}

bool DifferentialDrive::queueMoveToPose(const Pose &target, float linearSpeed, float turnSpeed) {
  InterruptGuard guard;
  if (_queueCount >= kDrivetrainQueueSize) return false;
  
  DrivetrainCommand cmd;
  cmd.type = DrivetrainCommandType::MoveToPose;
  cmd.pose.targetX = target.x;
  cmd.pose.targetY = target.y;
  cmd.pose.targetYaw = target.rot.getYawRads();
  cmd.pose.linearSpeed = linearSpeed;
  cmd.pose.turnSpeed = turnSpeed;
  
  _queue[_queueTail] = cmd;
  _queueTail = (_queueTail + 1) % kDrivetrainQueueSize;
  _queueCount++;
  return true;
}

bool DifferentialDrive::queueFollowTrajectory(const Trajectory &traj, float linearSpeed, float turnSpeed) {
  InterruptGuard guard;
  if (_queueCount >= kDrivetrainQueueSize) return false;
  
  DrivetrainCommand cmd;
  cmd.type = DrivetrainCommandType::FollowTrajectory;
  cmd.trajectory.points = traj.points;
  cmd.trajectory.count = traj.count;
  cmd.trajectory.linearSpeed = linearSpeed;
  cmd.trajectory.turnSpeed = turnSpeed;
  cmd.trajectory.currentIndex = 0;
  
  _queue[_queueTail] = cmd;
  _queueTail = (_queueTail + 1) % kDrivetrainQueueSize;
  _queueCount++;
  return true;
}

// Helpers
void DifferentialDrive::setWheelVelocities(float leftCmPerSec, float rightCmPerSec) {
  float leftSteps = leftCmPerSec * StepsPerCM;
  float rightSteps = rightCmPerSec * StepsPerCM;
  _left.commandVelocity(leftSteps);
  _right.commandVelocity(rightSteps);
}

float DifferentialDrive::normalizeAngle(float angle) {
  while (angle > PI_F) angle -= TWO_PI_F;
  while (angle < -PI_F) angle += TWO_PI_F;
  return angle;
}

float DifferentialDrive::degToRad(float deg) { return deg * kDegToRad; }
float DifferentialDrive::radToDeg(float rad) { return rad * kRadToDeg; }
