#include "drivetrain.h"

#include <cmath>

#if defined(__arm__) || defined(ARDUINO_ARCH_STM32)
#include <cmsis_gcc.h>
#endif

namespace {
constexpr double kDegToRad          = PI / 180.0;
constexpr double kRadToDeg          = 180.0 / PI;
constexpr double kVelocityEpsilon   = 1e-3;
constexpr double kPoseDistanceTolCm = 1.0;
constexpr double kPoseHeadingTolRad = 2.0 * kDegToRad;

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

DifferentialDrive::DifferentialDrive(Stepper &left,
                                     Stepper &right,
                                     Kalman  *filter,
                                     BNO     *imu)
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
    _targetHeading(0.0),
    _headingGain(2.0), // Default gain
    _turnGain(1.5),    // Default gain
    _subState(SubState::Init) {
  if (_filter) { _filter->reset(); }
}

void DifferentialDrive::setFilter(Kalman *filter) {
  _filter = filter;
  if (_filter) { _filter->reset(getPose()); }
}

void DifferentialDrive::setIMU(BNO *imu) { _imu = imu; }

void DifferentialDrive::setGains(double headingGain, double distanceGain) {
  _headingGain = headingGain;
  (void)distanceGain; // Unused for now
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
  if (_filter) { return _filter->getPoseEstimate(); }
  return Pose(); // Return zero pose if no filter
}

double DifferentialDrive::getHeading() const {
  if (_filter) { return _filter->getYaw(); }
  if (_imu) { return _imu->getRotation()->getYawRads(); }
  return 0.0;
}

void DifferentialDrive::update() {
  // 1. Update Filter/Pose
  long leftSteps  = _left.currentPosition();
  long rightSteps = _right.currentPosition();

  static long prevLeft = 0;
  static long prevRight = 0;
  static unsigned long prevMicros = 0;
  
  unsigned long now = micros();
  double dt = (now - prevMicros) * 1e-6;
  prevMicros = now;
  
  if (_filter) {
      _filter->predict(leftSteps - prevLeft, rightSteps - prevRight, dt);
      if (_imu && _imu->isReady()) {
          Rotation* rot = _imu->getRotation();
          if (rot) _filter->updateWithIMUYaw(rot->getYawRads());
      }
  }
  prevLeft = leftSteps;
  prevRight = rightSteps;

  // 2. Process Command
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
      
      if (_activeCommand.type == DrivetrainCommandType::DriveStraight) {
          _targetHeading = getHeading(); // Maintain current heading
      } else if (_activeCommand.type == DrivetrainCommandType::TurnDegrees) {
          _targetHeading = normalizeAngle(getHeading() + degToRad(_activeCommand.data.turn.degrees));
      } else if (_activeCommand.type == DrivetrainCommandType::FollowTrajectory) {
          _activeCommand.data.trajectory.currentIndex = 0;
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
    case DrivetrainCommandType::DriveStraight:
      handleDriveStraight();
      break;
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

void DifferentialDrive::handleDriveStraight() {
  double dist = getDistanceTraveled(_startLeftSteps, _startRightSteps, _left.currentPosition(), _right.currentPosition());
  double targetDist = std::abs(_activeCommand.data.straight.distCm);
  double error = targetDist - dist;

  if (error <= kPoseDistanceTolCm) {
    setWheelVelocities(0, 0);
    _isExecuting = false;
    return;
  }

  // Heading correction
  double currentHeading = getHeading();
  double headingError = normalizeAngle(_targetHeading - currentHeading);
  double correction = headingError * _headingGain; // Simple P-controller

  // Limit correction to avoid overpowering linear speed
  // correction is in cm/s effectively if we apply it to velocity
  
  double baseSpeed = _activeCommand.data.straight.speedCmPerSec;
  if (_activeCommand.data.straight.distCm < 0) baseSpeed = -baseSpeed;

  // Slow down near end
  if (error < 10.0) {
      baseSpeed *= (error / 10.0);
      if (std::abs(baseSpeed) < 2.0) baseSpeed = (baseSpeed > 0 ? 2.0 : -2.0);
  }

  double leftSpeed = baseSpeed - correction;
  double rightSpeed = baseSpeed + correction;

  setWheelVelocities(leftSpeed, rightSpeed);
}

void DifferentialDrive::handleTurnDegrees() {
  double currentHeading = getHeading();
  double error = normalizeAngle(_targetHeading - currentHeading);

  if (std::abs(error) <= kPoseHeadingTolRad) {
    setWheelVelocities(0, 0);
    _isExecuting = false;
    return;
  }

  double turnSpeed = error * _turnGain * kRadToDeg; // Convert to deg/s for scaling
  
  // Clamp speed
  double maxSpeed = std::abs(_activeCommand.data.turn.speedDegPerSec);
  if (turnSpeed > maxSpeed) turnSpeed = maxSpeed;
  if (turnSpeed < -maxSpeed) turnSpeed = -maxSpeed;
  
  // Minimum speed to overcome friction
  if (std::abs(turnSpeed) < 10.0) turnSpeed = (turnSpeed > 0 ? 10.0 : -10.0);

  // Convert deg/s to cm/s for wheels
  double wheelSpeed = degToRad(turnSpeed) * (DT_TRACK_WIDTH_CM / 2.0);

  setWheelVelocities(-wheelSpeed, wheelSpeed);
}

void DifferentialDrive::handleMoveToPose() {
    Pose currentPose = getPose();
    Pose targetPose = _activeCommand.data.pose.target;
    
    double dx = targetPose.x - currentPose.x;
    double dy = targetPose.y - currentPose.y;
    double dist = std::sqrt(dx*dx + dy*dy);
    double targetHeading = std::atan2(dy, dx);
    
    switch (_subState) {
        case SubState::Init:
            _subState = SubState::AlignToTarget;
            break;
            
        case SubState::AlignToTarget: {
            double headingError = normalizeAngle(targetHeading - currentPose.rot.getYawRads());
            if (std::abs(headingError) < kPoseHeadingTolRad) {
                _subState = SubState::DriveToTarget;
                _startLeftSteps = _left.currentPosition();
                _startRightSteps = _right.currentPosition();
            } else {
                // Turn logic
                double turnSpeed = headingError * _turnGain * kRadToDeg;
                double maxSpeed = _activeCommand.data.pose.turnSpeed;
                if (turnSpeed > maxSpeed) turnSpeed = maxSpeed;
                if (turnSpeed < -maxSpeed) turnSpeed = -maxSpeed;
                if (std::abs(turnSpeed) < 10.0) turnSpeed = (turnSpeed > 0 ? 10.0 : -10.0);
                double wheelSpeed = degToRad(turnSpeed) * (DT_TRACK_WIDTH_CM / 2.0);
                setWheelVelocities(-wheelSpeed, wheelSpeed);
            }
            break;
        }
            
        case SubState::DriveToTarget: {
            if (dist < kPoseDistanceTolCm) {
                _subState = SubState::FinalAlign;
            } else {
                // Drive logic with heading correction to target
                // Re-calculate target heading dynamically to drive straight to point
                double dynamicTargetHeading = std::atan2(dy, dx);
                double headingError = normalizeAngle(dynamicTargetHeading - currentPose.rot.getYawRads());
                
                // If we deviate too much, stop and realign
                if (std::abs(headingError) > degToRad(30)) {
                    _subState = SubState::AlignToTarget;
                    return;
                }
                
                double correction = headingError * _headingGain;
                double baseSpeed = _activeCommand.data.pose.linearSpeed;
                
                // Slow down
                if (dist < 10.0) baseSpeed *= (dist / 10.0);
                if (baseSpeed < 5.0) baseSpeed = 5.0;
                
                setWheelVelocities(baseSpeed - correction, baseSpeed + correction);
            }
            break;
        }
            
        case SubState::FinalAlign: {
            double finalHeading = targetPose.rot.getYawRads();
            double headingError = normalizeAngle(finalHeading - currentPose.rot.getYawRads());
             if (std::abs(headingError) < kPoseHeadingTolRad) {
                _subState = SubState::Done;
                setWheelVelocities(0, 0);
                _isExecuting = false;
            } else {
                // Turn logic
                double turnSpeed = headingError * _turnGain * kRadToDeg;
                double maxSpeed = _activeCommand.data.pose.turnSpeed;
                if (turnSpeed > maxSpeed) turnSpeed = maxSpeed;
                if (turnSpeed < -maxSpeed) turnSpeed = -maxSpeed;
                if (std::abs(turnSpeed) < 10.0) turnSpeed = (turnSpeed > 0 ? 10.0 : -10.0);
                double wheelSpeed = degToRad(turnSpeed) * (DT_TRACK_WIDTH_CM / 2.0);
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
    auto& trajData = _activeCommand.data.trajectory;
    
    if (trajData.currentIndex >= trajData.count) {
        _isExecuting = false;
        setWheelVelocities(0, 0);
        return;
    }
    
    // Create a temporary MoveToPose command for the current point
    // We can reuse the handleMoveToPose logic by setting up a fake command or refactoring.
    // To avoid code duplication, let's just run the logic inline or use a helper.
    // For simplicity, let's treat the current point as a target and use a state machine similar to MoveToPose.
    // But we need to persist state between updates.
    // We can use _subState for the current point.
    
    // Check if we just started this point
    // We need a way to know if we are initializing a new point.
    // Let's assume _subState is reset to Init when we increment index.
    
    Pose targetPose = trajData.points[trajData.currentIndex];
    
    // We can temporarily swap _activeCommand to MoveToPose to use handleMoveToPose?
    // No, that's messy. Let's just copy the logic or call a helper.
    // Actually, we can just use the same logic variables.
    
    // Let's implement a simplified version: Go to point, then next.
    // Note: Trajectory following usually implies continuous motion, not stop-and-turn at each point.
    // But "list of poses" suggests visiting them.
    // If we want continuous, we'd use Pure Pursuit or similar.
    // Given "revamp", let's stick to point-to-point for now unless requested otherwise.
    // The user said "follow a trajectory, which is just a list of poses".
    
    Pose currentPose = getPose();
    double dx = targetPose.x - currentPose.x;
    double dy = targetPose.y - currentPose.y;
    double dist = std::sqrt(dx*dx + dy*dy);
    
    // Logic:
    // 1. Turn to face point
    // 2. Drive to point
    // 3. If close enough, increment index
    
    double targetHeading = std::atan2(dy, dx);
    double headingError = normalizeAngle(targetHeading - currentPose.rot.getYawRads());
    
    if (dist < kPoseDistanceTolCm) {
        // Reached point
        trajData.currentIndex++;
        // Don't stop, just proceed to next point immediately
        return;
    }
    
    // If heading error is large, turn in place (or slow down turn)
    if (std::abs(headingError) > degToRad(20)) {
        // Turn in place
        double turnSpeed = headingError * _turnGain * kRadToDeg;
        double maxSpeed = trajData.turnSpeed;
        if (turnSpeed > maxSpeed) turnSpeed = maxSpeed;
        if (turnSpeed < -maxSpeed) turnSpeed = -maxSpeed;
        double wheelSpeed = degToRad(turnSpeed) * (DT_TRACK_WIDTH_CM / 2.0);
        setWheelVelocities(-wheelSpeed, wheelSpeed);
    } else {
        // Drive and correct
        double correction = headingError * _headingGain;
        double baseSpeed = trajData.linearSpeed;
        
        // Slow down if turning sharp
        baseSpeed *= std::cos(headingError);
        
        setWheelVelocities(baseSpeed - correction, baseSpeed + correction);
    }
}

// Queueing functions
bool DifferentialDrive::queueDriveStraight(double distanceCm, double speedCmPerSec) {
  InterruptGuard guard;
  if (_queueCount >= kDrivetrainQueueSize) return false;
  
  DrivetrainCommand cmd;
  cmd.type = DrivetrainCommandType::DriveStraight;
  cmd.data.straight.distCm = distanceCm;
  cmd.data.straight.speedCmPerSec = speedCmPerSec;
  
  _queue[_queueTail] = cmd;
  _queueTail = (_queueTail + 1) % kDrivetrainQueueSize;
  _queueCount++;
  return true;
}

bool DifferentialDrive::queueTurnDegrees(double degrees, double speedDegPerSec) {
  InterruptGuard guard;
  if (_queueCount >= kDrivetrainQueueSize) return false;
  
  DrivetrainCommand cmd;
  cmd.type = DrivetrainCommandType::TurnDegrees;
  cmd.data.turn.degrees = degrees;
  cmd.data.turn.speedDegPerSec = speedDegPerSec;
  
  _queue[_queueTail] = cmd;
  _queueTail = (_queueTail + 1) % kDrivetrainQueueSize;
  _queueCount++;
  return true;
}

bool DifferentialDrive::queueMoveToPose(const Pose &target, double linearSpeed, double turnSpeed) {
  InterruptGuard guard;
  if (_queueCount >= kDrivetrainQueueSize) return false;
  
  DrivetrainCommand cmd;
  cmd.type = DrivetrainCommandType::MoveToPose;
  cmd.data.pose.target = target;
  cmd.data.pose.linearSpeed = linearSpeed;
  cmd.data.pose.turnSpeed = turnSpeed;
  
  _queue[_queueTail] = cmd;
  _queueTail = (_queueTail + 1) % kDrivetrainQueueSize;
  _queueCount++;
  return true;
}

bool DifferentialDrive::queueFollowTrajectory(const Trajectory &traj, double linearSpeed, double turnSpeed) {
  InterruptGuard guard;
  if (_queueCount >= kDrivetrainQueueSize) return false;
  
  DrivetrainCommand cmd;
  cmd.type = DrivetrainCommandType::FollowTrajectory;
  cmd.data.trajectory.points = traj.points;
  cmd.data.trajectory.count = traj.count;
  cmd.data.trajectory.linearSpeed = linearSpeed;
  cmd.data.trajectory.turnSpeed = turnSpeed;
  cmd.data.trajectory.currentIndex = 0;
  
  _queue[_queueTail] = cmd;
  _queueTail = (_queueTail + 1) % kDrivetrainQueueSize;
  _queueCount++;
  return true;
}

// Helpers
void DifferentialDrive::setWheelVelocities(double leftCmPerSec, double rightCmPerSec) {
  float leftSteps = leftCmPerSec * StepsPerCM;
  float rightSteps = rightCmPerSec * StepsPerCM;
  _left.commandVelocity(leftSteps);
  _right.commandVelocity(rightSteps);
}

double DifferentialDrive::getDistanceTraveled(long startLeft, long startRight, long currentLeft, long currentRight) {
    double dl = cnv_stepsToCM(currentLeft - startLeft);
    double dr = cnv_stepsToCM(currentRight - startRight);
    return (dl + dr) / 2.0;
}

double DifferentialDrive::normalizeAngle(double angle) {
  while (angle > PI) angle -= TWO_PI;
  while (angle < -PI) angle += TWO_PI;
  return angle;
}

double DifferentialDrive::degToRad(double deg) { return deg * kDegToRad; }
double DifferentialDrive::radToDeg(double rad) { return rad * kRadToDeg; }
