#include "drivetrain.h"

#include <Arduino.h>

#include <algorithm>
#include <cmath>

namespace {
constexpr double kDegToRad          = PI / 180.0;
constexpr double kHeadingTolRad     = 1.0 * kDegToRad;
constexpr double kDistanceTolCm     = 0.5;
constexpr double kHeadingGainFactor = 0.5;
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
    _subAction(SubAction::None),
    _posePhase(PosePhase::None),
    _poseSequenceActive(false),
    _hasActiveCommand(false),
    _pose(),
    _activeCommand(),
    _queue(),
    _poseTarget(),
    _poseTargetHeading(0.0),
    _poseFinalYaw(0.0),
    _prevLeftSteps(left.currentPosition()),
    _prevRightSteps(right.currentPosition()),
    _baseLeftTarget(left.currentPosition()),
    _baseRightTarget(right.currentPosition()),
    _lastUpdateMicros(micros()),
    _turnMaxSpeedDegPerSec(0.0),
    _turnAccelDegPerSec2(0.0),
    _linearMaxSpeedCmPerSec(0.0),
    _linearAccelCmPerSec2(0.0),
    _headingHold(0.0) {
  if (_filter) { _pose = _filter->getPoseEstimate(); }
  _headingHold = currentYaw();
}

void DifferentialDrive::setFilter(Kalman *filter) {
  _filter = filter;
  if (_filter) {
    _pose         = _filter->getPoseEstimate();
    _headingHold  = currentYaw();
    _poseFinalYaw = _headingHold;
  }
}

void DifferentialDrive::setIMU(BNO *imu) { _imu = imu; }

void DifferentialDrive::resetPose(const Pose &pose) {
  if (_filter) { _filter->reset(pose); }

  _pose        = pose;
  _headingHold = currentYaw();

  _prevLeftSteps    = _left.currentPosition();
  _prevRightSteps   = _right.currentPosition();
  _baseLeftTarget   = _prevLeftSteps;
  _baseRightTarget  = _prevRightSteps;
  _lastUpdateMicros = micros();

  _mode               = Mode::Idle;
  _posePhase          = PosePhase::None;
  _poseSequenceActive = false;
  _hasActiveCommand   = false;
  _subAction          = SubAction::None;
  _queue.clear();
}

void DifferentialDrive::queueDriveStraight(double distanceCm,
                                           double maxSpeedCmPerSec,
                                           double accelCmPerSec2) {
  MoveCommand cmd;
  cmd.type      = MoveType::DriveStraight;
  cmd.distance  = distanceCm;
  cmd.linearMax = maxSpeedCmPerSec;
  cmd.linearAcc = accelCmPerSec2;
  _queue.push_back(cmd);
}

void DifferentialDrive::queueTurn(double degrees,
                                  double maxSpeedDegPerSec,
                                  double accelDegPerSec2) {
  MoveCommand cmd;
  cmd.type    = MoveType::Turn;
  cmd.degrees = degrees;
  cmd.turnMax = maxSpeedDegPerSec;
  cmd.turnAcc = accelDegPerSec2;
  _queue.push_back(cmd);
}

void DifferentialDrive::queueMoveToPose(const Pose &target,
                                        double      linearSpeedCmPerSec,
                                        double      accelCmPerSec2,
                                        double      turnSpeedDegPerSec,
                                        double      turnAccelDegPerSec2) {
  MoveCommand cmd;
  cmd.type       = MoveType::ToPose;
  cmd.targetPose = target;
  cmd.linearMax  = linearSpeedCmPerSec;
  cmd.linearAcc  = accelCmPerSec2;
  cmd.turnMax    = turnSpeedDegPerSec;
  cmd.turnAcc    = turnAccelDegPerSec2;
  _queue.push_back(cmd);
}

void DifferentialDrive::clearQueue() { _queue.clear(); }

void DifferentialDrive::stop() {
  _left.setTarget(_left.currentPosition());
  _right.setTarget(_right.currentPosition());

  _queue.clear();
  _mode               = Mode::Idle;
  _subAction          = SubAction::None;
  _posePhase          = PosePhase::None;
  _poseSequenceActive = false;
  _hasActiveCommand   = false;
  _headingHold        = currentYaw();
}

void DifferentialDrive::update() {
  unsigned long now = micros();
  double        dt  = (_lastUpdateMicros > 0)
                        ? static_cast<double>(now - _lastUpdateMicros) / 1e6
                        : 0.0;
  _lastUpdateMicros = now;

  long leftSteps  = _left.currentPosition();
  long rightSteps = _right.currentPosition();

  long deltaLeft  = leftSteps - _prevLeftSteps;
  long deltaRight = rightSteps - _prevRightSteps;

  updateFilter(deltaLeft, deltaRight, dt);

  _prevLeftSteps  = leftSteps;
  _prevRightSteps = rightSteps;

  bool leftBusy  = _left.isBusy();
  bool rightBusy = _right.isBusy();

  if (_subAction == SubAction::Driving) { applyHeadingCorrection(); }

  if (_mode == Mode::MoveToPose && _poseSequenceActive) {
    updatePoseSequence(leftBusy, rightBusy);
  } else if (_hasActiveCommand && !leftBusy && !rightBusy) {
    finishActiveCommand();
  }

  if (!_hasActiveCommand && !_queue.empty() && !_left.isBusy() &&
      !_right.isBusy()) {
    startNextCommand();
  }
}

bool DifferentialDrive::isBusy() const {
  return _left.isBusy() || _right.isBusy() || _hasActiveCommand ||
         !_queue.empty();
}

size_t DifferentialDrive::queuedMoves() const {
  return _queue.size() +
         (_hasActiveCommand ? static_cast<size_t>(1) : static_cast<size_t>(0));
}

void DifferentialDrive::startNextCommand() {
  while (!_queue.empty()) {
    MoveCommand cmd = _queue.front();
    _queue.pop_front();

    bool started = false;
    switch (cmd.type) {
      case MoveType::DriveStraight:
        started = startDrive(cmd.distance, cmd.linearMax, cmd.linearAcc);
        break;
      case MoveType::Turn:
        started = startTurn(cmd.degrees, cmd.turnMax, cmd.turnAcc);
        break;
      case MoveType::ToPose: started = startPoseCommand(cmd); break;
    }

    if (started) {
      _activeCommand    = cmd;
      _hasActiveCommand = true;
      return;
    }
  }
}

bool DifferentialDrive::startDrive(double distanceCm,
                                   double maxSpeedCmPerSec,
                                   double accelCmPerSec2) {
  long stepDelta = cnv_CMToSteps(distanceCm);
  if (stepDelta == 0) { return false; }

  double requestedSpeedSteps =
    (maxSpeedCmPerSec > 0.0) ? maxSpeedCmPerSec * StepsPerCM : 0.0;
  double requestedAccelSteps =
    (accelCmPerSec2 > 0.0) ? accelCmPerSec2 * StepsPerCM : 0.0;

  int32_t maxSpeedSteps =
    clampSpeedSteps(requestedSpeedSteps, static_cast<double>(MOTOR_MAX_SPEED));
  int32_t accelSteps =
    clampSpeedSteps(requestedAccelSteps, static_cast<double>(MOTOR_MAX_ACCEL));

  _linearMaxSpeedCmPerSec = maxSpeedCmPerSec;
  _linearAccelCmPerSec2   = accelCmPerSec2;

  issueDriveCommand(
    stepDelta, maxSpeedSteps, accelSteps, Mode::DrivingStraight);
  return true;
}

bool DifferentialDrive::startTurn(double degrees,
                                  double maxSpeedDegPerSec,
                                  double accelDegPerSec2) {
  double radians = degrees * kDegToRad;
  if (std::fabs(radians) < 1e-6) { return false; }

  double arcCm     = radians * (DT_TRACK_WIDTH_CM * 0.5);
  long   stepDelta = cnv_CMToSteps(arcCm);
  if (stepDelta == 0) { return false; }

  double requestedSpeedSteps = (maxSpeedDegPerSec > 0.0)
                                 ? std::fabs(maxSpeedDegPerSec) * kDegToRad *
                                     (DT_TRACK_WIDTH_CM * 0.5) * StepsPerCM
                                 : 0.0;
  double requestedAccelSteps = (accelDegPerSec2 > 0.0)
                                 ? std::fabs(accelDegPerSec2) * kDegToRad *
                                     (DT_TRACK_WIDTH_CM * 0.5) * StepsPerCM
                                 : 0.0;

  int32_t maxSpeedSteps =
    clampSpeedSteps(requestedSpeedSteps, static_cast<double>(MOTOR_MAX_SPEED));
  int32_t accelSteps =
    clampSpeedSteps(requestedAccelSteps, static_cast<double>(MOTOR_MAX_ACCEL));

  _turnMaxSpeedDegPerSec = maxSpeedDegPerSec;
  _turnAccelDegPerSec2   = accelDegPerSec2;

  issueTurnCommand(stepDelta, maxSpeedSteps, accelSteps, Mode::Turning);
  return true;
}

bool DifferentialDrive::startPoseCommand(const MoveCommand &command) {
  _poseTarget   = command.targetPose;
  _poseFinalYaw = _poseTarget.rot.getYawRads();

  double dx          = _poseTarget.x - _pose.x;
  double dy          = _poseTarget.y - _pose.y;
  double distance    = std::sqrt(dx * dx + dy * dy);
  _poseTargetHeading = (distance > 1e-6) ? std::atan2(dy, dx) : currentYaw();

  double finalError = normalizeAngle(_poseFinalYaw - currentYaw());
  if (distance <= kDistanceTolCm && std::fabs(finalError) <= kHeadingTolRad) {
    return false;
  }

  _linearMaxSpeedCmPerSec = command.linearMax;
  _linearAccelCmPerSec2   = command.linearAcc;
  _turnMaxSpeedDegPerSec  = command.turnMax;
  _turnAccelDegPerSec2    = command.turnAcc;

  _mode               = Mode::MoveToPose;
  _subAction          = SubAction::None;
  _posePhase          = PosePhase::RotateToHeading;
  _poseSequenceActive = true;

  updatePoseSequence(false, false);
  return true;
}

void DifferentialDrive::finishActiveCommand() {
  _hasActiveCommand   = false;
  _mode               = Mode::Idle;
  _subAction          = SubAction::None;
  _posePhase          = PosePhase::None;
  _poseSequenceActive = false;
  _headingHold        = currentYaw();
  _activeCommand      = MoveCommand();
}

void DifferentialDrive::applyHeadingCorrection() {
  if (!_filter) { return; }

  double yawError     = normalizeAngle(_headingHold - currentYaw());
  double correctionCm = std::clamp(
    yawError * (DT_TRACK_WIDTH_CM * 0.5) * kHeadingGainFactor, -5.0, 5.0);

  long correctionSteps = cnv_CMToSteps(correctionCm);

  long desiredLeft  = _baseLeftTarget - correctionSteps;
  long desiredRight = _baseRightTarget + correctionSteps;

  if (desiredLeft != _left.targetPosition()) { _left.setTarget(desiredLeft); }
  if (desiredRight != _right.targetPosition()) {
    _right.setTarget(desiredRight);
  }
}

void DifferentialDrive::updatePoseSequence(bool leftBusy, bool rightBusy) {
  if (!_poseSequenceActive) { return; }

  auto finishSequence = [this]() { finishActiveCommand(); };

  switch (_posePhase) {
    case PosePhase::None: finishSequence(); return;

    case PosePhase::RotateToHeading: {
      if (_subAction == SubAction::Turning) {
        if (!leftBusy && !rightBusy) {
          _subAction = SubAction::None;
        } else {
          return;
        }
      }

      if (_subAction != SubAction::None) { return; }

      double dx       = _poseTarget.x - _pose.x;
      double dy       = _poseTarget.y - _pose.y;
      double distance = std::sqrt(dx * dx + dy * dy);
      if (distance <= kDistanceTolCm) {
        _posePhase = PosePhase::FinalTurn;
        return;
      }

      _poseTargetHeading = std::atan2(dy, dx);
      double yawError    = normalizeAngle(_poseTargetHeading - currentYaw());

      if (std::fabs(yawError) > kHeadingTolRad) {
        double radians   = yawError;
        double arcCm     = radians * (DT_TRACK_WIDTH_CM * 0.5);
        long   stepDelta = cnv_CMToSteps(arcCm);
        if (stepDelta == 0) {
          _posePhase = PosePhase::DriveStraight;
          return;
        }

        double requestedSpeedSteps =
          (_turnMaxSpeedDegPerSec > 0.0)
            ? std::fabs(_turnMaxSpeedDegPerSec) * kDegToRad *
                (DT_TRACK_WIDTH_CM * 0.5) * StepsPerCM
            : 0.0;
        double requestedAccelSteps =
          (_turnAccelDegPerSec2 > 0.0)
            ? std::fabs(_turnAccelDegPerSec2) * kDegToRad *
                (DT_TRACK_WIDTH_CM * 0.5) * StepsPerCM
            : 0.0;

        int32_t maxSpeedSteps = clampSpeedSteps(
          requestedSpeedSteps, static_cast<double>(MOTOR_MAX_SPEED));
        int32_t accelSteps = clampSpeedSteps(
          requestedAccelSteps, static_cast<double>(MOTOR_MAX_ACCEL));

        issueTurnCommand(
          stepDelta, maxSpeedSteps, accelSteps, Mode::MoveToPose);
        return;
      }

      long stepDelta = cnv_CMToSteps(distance);
      if (stepDelta == 0) {
        _posePhase = PosePhase::FinalTurn;
        return;
      }

      double requestedSpeedSteps = (_linearMaxSpeedCmPerSec > 0.0)
                                     ? _linearMaxSpeedCmPerSec * StepsPerCM
                                     : 0.0;
      double requestedAccelSteps = (_linearAccelCmPerSec2 > 0.0)
                                     ? _linearAccelCmPerSec2 * StepsPerCM
                                     : 0.0;

      int32_t maxSpeedSteps = clampSpeedSteps(
        requestedSpeedSteps, static_cast<double>(MOTOR_MAX_SPEED));
      int32_t accelSteps = clampSpeedSteps(
        requestedAccelSteps, static_cast<double>(MOTOR_MAX_ACCEL));

      issueDriveCommand(stepDelta, maxSpeedSteps, accelSteps, Mode::MoveToPose);
      _posePhase = PosePhase::DriveStraight;
      return;
    }

    case PosePhase::DriveStraight: {
      if (_subAction == SubAction::Driving) {
        if (!leftBusy && !rightBusy) {
          _subAction = SubAction::None;
        } else {
          return;
        }
      }

      if (_subAction != SubAction::None) { return; }

      double finalError = normalizeAngle(_poseFinalYaw - currentYaw());
      if (std::fabs(finalError) <= kHeadingTolRad) {
        finishSequence();
        return;
      }

      double radians   = finalError;
      double arcCm     = radians * (DT_TRACK_WIDTH_CM * 0.5);
      long   stepDelta = cnv_CMToSteps(arcCm);
      if (stepDelta == 0) {
        finishSequence();
        return;
      }

      double requestedSpeedSteps = (_turnMaxSpeedDegPerSec > 0.0)
                                     ? std::fabs(_turnMaxSpeedDegPerSec) *
                                         kDegToRad * (DT_TRACK_WIDTH_CM * 0.5) *
                                         StepsPerCM
                                     : 0.0;
      double requestedAccelSteps = (_turnAccelDegPerSec2 > 0.0)
                                     ? std::fabs(_turnAccelDegPerSec2) *
                                         kDegToRad * (DT_TRACK_WIDTH_CM * 0.5) *
                                         StepsPerCM
                                     : 0.0;

      int32_t maxSpeedSteps = clampSpeedSteps(
        requestedSpeedSteps, static_cast<double>(MOTOR_MAX_SPEED));
      int32_t accelSteps = clampSpeedSteps(
        requestedAccelSteps, static_cast<double>(MOTOR_MAX_ACCEL));

      issueTurnCommand(stepDelta, maxSpeedSteps, accelSteps, Mode::MoveToPose);
      _posePhase = PosePhase::FinalTurn;
      return;
    }

    case PosePhase::FinalTurn: {
      if (_subAction == SubAction::Turning) {
        if (!leftBusy && !rightBusy) {
          _subAction = SubAction::None;
        } else {
          return;
        }
      }

      if (_subAction != SubAction::None) { return; }

      double finalError = normalizeAngle(_poseFinalYaw - currentYaw());
      if (std::fabs(finalError) <= kHeadingTolRad) {
        finishSequence();
        return;
      }

      double radians   = finalError;
      double arcCm     = radians * (DT_TRACK_WIDTH_CM * 0.5);
      long   stepDelta = cnv_CMToSteps(arcCm);
      if (stepDelta == 0) {
        finishSequence();
        return;
      }

      double requestedSpeedSteps = (_turnMaxSpeedDegPerSec > 0.0)
                                     ? std::fabs(_turnMaxSpeedDegPerSec) *
                                         kDegToRad * (DT_TRACK_WIDTH_CM * 0.5) *
                                         StepsPerCM
                                     : 0.0;
      double requestedAccelSteps = (_turnAccelDegPerSec2 > 0.0)
                                     ? std::fabs(_turnAccelDegPerSec2) *
                                         kDegToRad * (DT_TRACK_WIDTH_CM * 0.5) *
                                         StepsPerCM
                                     : 0.0;

      int32_t maxSpeedSteps = clampSpeedSteps(
        requestedSpeedSteps, static_cast<double>(MOTOR_MAX_SPEED));
      int32_t accelSteps = clampSpeedSteps(
        requestedAccelSteps, static_cast<double>(MOTOR_MAX_ACCEL));

      issueTurnCommand(stepDelta, maxSpeedSteps, accelSteps, Mode::MoveToPose);
      return;
    }
  }
}

void DifferentialDrive::issueDriveCommand(long    stepDelta,
                                          int32_t maxSpeedSteps,
                                          int32_t accelSteps,
                                          Mode    commandMode) {
  _mode        = commandMode;
  _subAction   = SubAction::Driving;
  _headingHold = currentYaw();

  _baseLeftTarget  = _left.currentPosition() + stepDelta;
  _baseRightTarget = _right.currentPosition() + stepDelta;

  Serial1.println("Issuing drive command: steps=");
  Serial1.println(stepDelta);

  _left.moveBy(stepDelta, maxSpeedSteps, accelSteps);
  _right.moveBy(stepDelta, maxSpeedSteps, accelSteps);
}

void DifferentialDrive::issueTurnCommand(long    stepDelta,
                                         int32_t maxSpeedSteps,
                                         int32_t accelSteps,
                                         Mode    commandMode) {
  _mode        = commandMode;
  _subAction   = SubAction::Turning;
  _headingHold = currentYaw();

  _baseLeftTarget  = _left.currentPosition() - stepDelta;
  _baseRightTarget = _right.currentPosition() + stepDelta;

  _left.moveBy(-stepDelta, maxSpeedSteps, accelSteps);
  _right.moveBy(stepDelta, maxSpeedSteps, accelSteps);
}

void DifferentialDrive::updateFilter(long   deltaLeft,
                                     long   deltaRight,
                                     double dtSeconds) {
  if (!_filter) {
    double leftCm   = cnv_stepsToCM(deltaLeft);
    double rightCm  = cnv_stepsToCM(deltaRight);
    double dCenter  = 0.5 * (leftCm + rightCm);
    double dTheta   = (rightCm - leftCm) / DT_TRACK_WIDTH_CM;
    double theta    = _pose.rot.getYawRads();
    double thetaMid = theta + 0.5 * dTheta;

    _pose.x += dCenter * std::cos(thetaMid);
    _pose.y += dCenter * std::sin(thetaMid);
    _pose.rot.setYawRads(normalizeAngle(theta + dTheta));
    return;
  }

  if (deltaLeft != 0 || deltaRight != 0 || dtSeconds > 0.0) {
    _filter->predict(deltaLeft, deltaRight, dtSeconds);
  }

  Rotation *imuRot = _imu->getRotation();
  if (imuRot) { _filter->updateWithIMUYaw(imuRot->getYawRads()); }

  _pose = _filter->getPoseEstimate();
}

double DifferentialDrive::currentYaw() const { return _pose.rot.yaw; }

double DifferentialDrive::normalizeAngle(double angle) {
  while (angle > PI) { angle -= TWO_PI; }
  while (angle < -PI) { angle += TWO_PI; }

  return angle;
}

int32_t DifferentialDrive::clampSpeedSteps(double  requested,
                                           double  defaultValue,
                                           int32_t minValue) {
  double value = (requested > 0.0) ? requested : defaultValue;
  if (value < static_cast<double>(minValue)) {
    value = static_cast<double>(minValue);
  }
  int32_t steps = static_cast<int32_t>(std::lround(value));
  if (steps < minValue) { steps = minValue; }
  return steps;
}
