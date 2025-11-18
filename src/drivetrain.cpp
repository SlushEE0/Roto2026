#include <Arduino.h>
#include <cmath>

#include "drivetrain.h"

namespace {
constexpr double kDegToRad          = PI / 180.0;
constexpr double kRadToDeg          = 180.0 / PI;
constexpr double kVelocityEpsilon   = 1e-3;
constexpr double kPoseDistanceTolCm = 0.5;
constexpr double kPoseHeadingTolRad = 1.0 * kDegToRad;
constexpr double kHeadingHoldGain   = 0.4;
} // namespace

DifferentialDrive::DifferentialDrive(Stepper &left,
                                     Stepper &right,
                                     Kalman  *filter,
                                     BNO     *imu)
  : _left(left),
    _right(right),
    _filter(filter),
    _imu(imu),
    _mode(Mode::Idle),
    _poseStage(PoseStage::Idle),
    _poseModeActive(false),
    _pose(),
    _poseTarget(),
    _linearCmd(0.0),
    _angularCmd(0.0),
    _headingHold(0.0),
    _headingGain(kHeadingHoldGain),
    _poseLinearSpeed(0.0),
    _poseTurnSpeed(0.0),
    _poseToleranceCm(kPoseDistanceTolCm),
    _poseToleranceRad(kPoseHeadingTolRad),
    _leftTarget(left.currentPosition()),
    _rightTarget(right.currentPosition()),
    _prevLeftSteps(left.currentPosition()),
    _prevRightSteps(right.currentPosition()),
    _positionSpeedSteps(0.0f),
    _lastUpdateMicros(micros()) {
  if (_filter) { _filter->reset(_pose); }
  refreshHeadingHold();
}

void DifferentialDrive::setFilter(Kalman *filter) {
  _filter = filter;
  if (_filter) { _filter->reset(_pose); }
}

void DifferentialDrive::setIMU(BNO *imu) { _imu = imu; }

void DifferentialDrive::resetPose(const Pose &pose) {
  _pose = pose;
  if (_filter) { _filter->reset(pose); }
  _prevLeftSteps    = _left.currentPosition();
  _prevRightSteps   = _right.currentPosition();
  _lastUpdateMicros = micros();
  refreshHeadingHold();
}

void DifferentialDrive::commandVelocity(double linearCmPerSec,
                                        double angularDegPerSec) {
  _poseModeActive = false;
  _poseStage      = PoseStage::Idle;

  if (std::fabs(linearCmPerSec) < kVelocityEpsilon &&
      std::fabs(angularDegPerSec) < kVelocityEpsilon) {
    stop(false);
    return;
  }

  _linearCmd  = linearCmPerSec;
  _angularCmd = angularDegPerSec;
  _mode       = Mode::Velocity;

  double halfTrack     = DT_TRACK_WIDTH_CM * 0.5;
  double angularRad    = degToRad(angularDegPerSec);
  double leftCmPerSec  = linearCmPerSec - angularRad * halfTrack;
  double rightCmPerSec = linearCmPerSec + angularRad * halfTrack;

  if (std::fabs(angularDegPerSec) < kVelocityEpsilon) {
    refreshHeadingHold();
    double yawError   = normalizeAngle(_headingHold - currentYaw());
    double correction = yawError * _headingGain;
    leftCmPerSec -= correction * halfTrack;
    rightCmPerSec += correction * halfTrack;
  } else {
    _headingHold = currentYaw();
  }

  applyVelocityCommand(leftCmPerSec, rightCmPerSec);
}

void DifferentialDrive::commandWheelVelocities(double leftCmPerSec,
                                               double rightCmPerSec) {
  _poseModeActive = false;
  _poseStage      = PoseStage::Idle;

  if (std::fabs(leftCmPerSec) < kVelocityEpsilon &&
      std::fabs(rightCmPerSec) < kVelocityEpsilon) {
    stop(false);
    return;
  }

  double linear     = 0.5 * (leftCmPerSec + rightCmPerSec);
  double angularRad = (rightCmPerSec - leftCmPerSec) / DT_TRACK_WIDTH_CM;

  _linearCmd   = linear;
  _angularCmd  = angularRad * kRadToDeg;
  _mode        = Mode::Velocity;
  _headingHold = currentYaw();

  applyVelocityCommand(leftCmPerSec, rightCmPerSec);
}

void DifferentialDrive::driveStraight(double distanceCm, double speedCmPerSec) {
  _poseModeActive = false;
  _poseStage      = PoseStage::Idle;
  _mode           = Mode::Position;
  _linearCmd      = 0.0;
  _angularCmd     = 0.0;

  if (speedCmPerSec <= 0.0) {
    speedCmPerSec = (_poseLinearSpeed > 0.0) ? _poseLinearSpeed : 20.0;
  }

  float speedSteps = static_cast<float>(std::fabs(speedCmPerSec) * StepsPerCM);
  if (speedSteps <= 0.0f) { speedSteps = static_cast<float>(MOTOR_MAX_SPEED); }

  long stepDelta      = cnv_CMToSteps(distanceCm);
  _leftTarget         = _left.currentPosition() + stepDelta;
  _rightTarget        = _right.currentPosition() + stepDelta;
  _positionSpeedSteps = speedSteps;

  _left.commandPosition(_leftTarget, speedSteps);
  _right.commandPosition(_rightTarget, speedSteps);
}

void DifferentialDrive::turnDegrees(double degrees, double speedDegPerSec) {
  _poseModeActive = false;
  _poseStage      = PoseStage::Idle;
  _mode           = Mode::Position;
  _linearCmd      = 0.0;
  _angularCmd     = 0.0;

  double radians     = degToRad(degrees);
  double halfTrack   = DT_TRACK_WIDTH_CM * 0.5;
  double wheelTravel = radians * halfTrack;
  long   stepDelta   = cnv_CMToSteps(wheelTravel);

  double angularSpeedRad = degToRad(std::fabs(speedDegPerSec));
  double wheelSpeedCm    = angularSpeedRad * halfTrack;
  float  speedSteps      = static_cast<float>(wheelSpeedCm * StepsPerCM);
  if (speedSteps <= 0.0f) { speedSteps = static_cast<float>(MOTOR_MAX_SPEED); }

  _leftTarget         = _left.currentPosition() - stepDelta;
  _rightTarget        = _right.currentPosition() + stepDelta;
  _positionSpeedSteps = speedSteps;

  _left.commandPosition(_leftTarget, speedSteps);
  _right.commandPosition(_rightTarget, speedSteps);
}

void DifferentialDrive::moveToPose(const Pose &target,
                                   double      linearSpeedCmPerSec,
                                   double      turnSpeedDegPerSec) {
  _poseTarget      = target;
  _poseLinearSpeed = std::fabs(linearSpeedCmPerSec);
  if (_poseLinearSpeed <= 0.0) { _poseLinearSpeed = 20.0; }
  _poseTurnSpeed = std::fabs(turnSpeedDegPerSec);
  if (_poseTurnSpeed <= 0.0) { _poseTurnSpeed = 45.0; }

  _poseModeActive = true;
  _poseStage      = PoseStage::RotateToHeading;
  _mode           = Mode::Idle;
  _linearCmd      = 0.0;
  _angularCmd     = 0.0;
}

void DifferentialDrive::stop(bool disableDrivers) {
  _linearCmd      = 0.0;
  _angularCmd     = 0.0;
  _mode           = Mode::Idle;
  _poseModeActive = false;
  _poseStage      = PoseStage::Idle;

  _left.stop(disableDrivers);
  _right.stop(disableDrivers);
  if (!disableDrivers) {
    _left.enable();
    _right.enable();
  }
  refreshHeadingHold();
}

void DifferentialDrive::update() {
  unsigned long now       = micros();
  double        dtSeconds = 0.0;
  if (now >= _lastUpdateMicros) {
    dtSeconds = static_cast<double>(now - _lastUpdateMicros) * 1e-6;
  }
  _lastUpdateMicros = now;

  long leftSteps  = _left.currentPosition();
  long rightSteps = _right.currentPosition();
  long deltaLeft  = leftSteps - _prevLeftSteps;
  long deltaRight = rightSteps - _prevRightSteps;

  if (deltaLeft != 0 || deltaRight != 0) {
    updatePose(deltaLeft, deltaRight, dtSeconds);
  } else if (_filter) {
    _pose = _filter->getPoseEstimate();
  }

  _prevLeftSteps  = leftSteps;
  _prevRightSteps = rightSteps;

  if (_mode == Mode::Position) {
    if (!_left.isBusy() && !_right.isBusy()) { _mode = Mode::Idle; }
  } else if (_mode == Mode::Velocity) {
    double halfTrack     = DT_TRACK_WIDTH_CM * 0.5;
    double angularRad    = degToRad(_angularCmd);
    double leftCmPerSec  = _linearCmd - angularRad * halfTrack;
    double rightCmPerSec = _linearCmd + angularRad * halfTrack;

    if (std::fabs(_angularCmd) < kVelocityEpsilon && _headingGain > 0.0) {
      double yawError   = normalizeAngle(_headingHold - currentYaw());
      double correction = yawError * _headingGain;
      leftCmPerSec -= correction * halfTrack;
      rightCmPerSec += correction * halfTrack;
    }

    if (std::fabs(leftCmPerSec) < kVelocityEpsilon &&
        std::fabs(rightCmPerSec) < kVelocityEpsilon) {
      stop(false);
    } else {
      applyVelocityCommand(leftCmPerSec, rightCmPerSec);
    }
  }

  if (_poseModeActive) { updatePoseSequence(); }
}

bool DifferentialDrive::isBusy() const {
  if (_poseModeActive) { return true; }
  if (_mode == Mode::Position) { return _left.isBusy() || _right.isBusy(); }
  if (_mode == Mode::Velocity) {
    return (std::fabs(_linearCmd) >= kVelocityEpsilon) ||
           (std::fabs(_angularCmd) >= kVelocityEpsilon);
  }
  return false;
}

void DifferentialDrive::applyVelocityCommand(double leftCmPerSec,
                                             double rightCmPerSec) {
  float leftStepsPerSec  = static_cast<float>(leftCmPerSec * StepsPerCM);
  float rightStepsPerSec = static_cast<float>(rightCmPerSec * StepsPerCM);

  if (MOTOR_MAX_SPEED > 0) {
    float maxSpeed = static_cast<float>(MOTOR_MAX_SPEED);
    if (leftStepsPerSec > maxSpeed) leftStepsPerSec = maxSpeed;
    if (leftStepsPerSec < -maxSpeed) leftStepsPerSec = -maxSpeed;
    if (rightStepsPerSec > maxSpeed) rightStepsPerSec = maxSpeed;
    if (rightStepsPerSec < -maxSpeed) rightStepsPerSec = -maxSpeed;
  }

  _left.commandVelocity(leftStepsPerSec);
  _right.commandVelocity(rightStepsPerSec);
}

void DifferentialDrive::updatePose(long   deltaLeft,
                                   long   deltaRight,
                                   double dtSeconds) {
  if (_filter) {
    _filter->predict(deltaLeft, deltaRight, dtSeconds);
    if (_imu && _imu->isReady()) {
      Rotation *rot = _imu->getRotation();
      if (rot) { _filter->updateWithIMUYaw(rot->getYawRads()); }
    }
    _pose = _filter->getPoseEstimate();
    return;
  }

  double dl      = cnv_stepsToCM(deltaLeft);
  double dr      = cnv_stepsToCM(deltaRight);
  double dCenter = 0.5 * (dl + dr);
  double dTheta  = (dr - dl) / DT_TRACK_WIDTH_CM;

  double theta    = _pose.rot.getYawRads();
  double thetaMid = theta + 0.5 * dTheta;
  _pose.x += dCenter * std::cos(thetaMid);
  _pose.y += dCenter * std::sin(thetaMid);
  _pose.rot.setYawRads(normalizeAngle(theta + dTheta));

  if (_imu && _imu->isReady()) {
    Rotation *rot = _imu->getRotation();
    if (rot) { _pose.rot.setYawRads(rot->getYawRads()); }
  }
}

void DifferentialDrive::updatePoseSequence() {
  if (!_poseModeActive) { return; }
  if (_left.isBusy() || _right.isBusy()) { return; }

  switch (_poseStage) {
    case PoseStage::RotateToHeading: {
      double dx       = _poseTarget.x - _pose.x;
      double dy       = _poseTarget.y - _pose.y;
      double distance = std::sqrt(dx * dx + dy * dy);
      if (distance <= _poseToleranceCm) {
        _poseStage = PoseStage::FinalRotate;
        return;
      }

      double desiredHeading = std::atan2(dy, dx);
      double yaw            = currentYaw();
      double headingError   = normalizeAngle(desiredHeading - yaw);

      if (std::fabs(headingError) <= _poseToleranceRad) {
        _poseStage = PoseStage::Translate;
        return;
      }

      turnDegrees(headingError * kRadToDeg, _poseTurnSpeed);
      break;
    }

    case PoseStage::Translate: {
      double dx       = _poseTarget.x - _pose.x;
      double dy       = _poseTarget.y - _pose.y;
      double distance = std::sqrt(dx * dx + dy * dy);
      if (distance <= _poseToleranceCm) {
        _poseStage = PoseStage::FinalRotate;
        return;
      }

      double desiredHeading = std::atan2(dy, dx);
      double yaw            = currentYaw();
      double headingError   = normalizeAngle(desiredHeading - yaw);
      if (std::fabs(headingError) > _poseToleranceRad) {
        _poseStage = PoseStage::RotateToHeading;
        return;
      }

      driveStraight(distance, _poseLinearSpeed);
      break;
    }

    case PoseStage::FinalRotate: {
      double targetYaw = _poseTarget.rot.getYawRads();
      double yaw       = currentYaw();
      double error     = normalizeAngle(targetYaw - yaw);

      if (std::fabs(error) <= _poseToleranceRad) {
        _poseModeActive = false;
        _poseStage      = PoseStage::Idle;
        stop(false);
        return;
      }

      turnDegrees(error * kRadToDeg, _poseTurnSpeed);
      break;
    }

    case PoseStage::Idle:
    default:
      _poseModeActive = false;
      _poseStage      = PoseStage::Idle;
      break;
  }
}

void DifferentialDrive::refreshHeadingHold() { _headingHold = currentYaw(); }

double DifferentialDrive::currentYaw() {
  if (_filter) { return _filter->getYaw(); }
  return _pose.rot.getYawRads();
}

double DifferentialDrive::degToRad(double deg) { return deg * kDegToRad; }

double DifferentialDrive::normalizeAngle(double angle) {
  while (angle > PI) angle -= TWO_PI;
  while (angle < -PI) angle += TWO_PI;
  return angle;
}
