#include "drivetrain.h"

#include <Arduino.h>

#include <algorithm>
#include <cmath>

namespace {
constexpr double kDegToRad          = PI / 180.0;
constexpr double kRadToDeg          = 180.0 / PI;
constexpr double kHeadingTolRad     = 1.0 * kDegToRad;
constexpr double kDistanceTolCm     = 0.5;
constexpr double kHeadingGainFactor = 0.5; // Tunable heading correction gain
} // namespace

DifferentialDrive::DifferentialDrive(Stepper &left,
                                     Stepper &right,
                                     Kalman  *filter,
                                     BNO     *imu)
  : m_left(left),
    m_right(right),
    m_filter(filter),
    m_imu(imu),
    m_mode(Mode::Idle),
    m_subAction(SubAction::None),
    m_movePhase(MovePhase::None),
    m_sequenceActive(false),
    m_pose(),
    m_moveTargetPose(),
    m_headingHold(0.0),
    m_targetYaw(0.0),
    m_moveTargetHeading(0.0),
    m_moveFinalYaw(0.0),
    m_prevLeftSteps(left.currentPosition()),
    m_prevRightSteps(right.currentPosition()),
    m_baseLeftTarget(left.currentPosition()),
    m_baseRightTarget(right.currentPosition()),
    m_lastUpdateMicros(micros()),
    m_turnMaxSpeedDegPerSec(0.0),
    m_turnAccelDegPerSec2(0.0),
    m_linearMaxSpeedCmPerSec(0.0),
    m_linearAccelCmPerSec2(0.0) {
  if (m_filter) { m_pose = m_filter->getPoseEstimate(); }
  m_headingHold = currentYaw();
  m_targetYaw   = m_headingHold;
}

void DifferentialDrive::setFilter(Kalman *filter) {
  m_filter = filter;
  if (m_filter) {
    m_pose         = m_filter->getPoseEstimate();
    m_headingHold  = currentYaw();
    m_targetYaw    = m_headingHold;
    m_moveFinalYaw = m_headingHold;
  }
}

void DifferentialDrive::setIMU(BNO *imu) { m_imu = imu; }

void DifferentialDrive::resetPose(const Pose &pose) {
  if (m_filter) { m_filter->reset(pose); }
  m_pose        = pose;
  m_headingHold = currentYaw();
  m_targetYaw   = m_headingHold;

  m_prevLeftSteps    = m_left.currentPosition();
  m_prevRightSteps   = m_right.currentPosition();
  m_baseLeftTarget   = m_prevLeftSteps;
  m_baseRightTarget  = m_prevRightSteps;
  m_lastUpdateMicros = micros();

  m_mode           = Mode::Idle;
  m_movePhase      = MovePhase::None;
  m_sequenceActive = false;
  m_subAction      = SubAction::None;
}

void DifferentialDrive::driveStraight(double distanceCm,
                                      double maxSpeedCmPerSec,
                                      double accelCmPerSec2) {
  long stepDelta = cmToSteps(distanceCm);
  if (stepDelta == 0) return;

  double requestedSpeedSteps =
    (maxSpeedCmPerSec > 0.0) ? maxSpeedCmPerSec * StepsPerCM : 0.0;
  double requestedAccelSteps =
    (accelCmPerSec2 > 0.0) ? accelCmPerSec2 * StepsPerCM : 0.0;

  int32_t maxSpeedSteps =
    clampSpeedSteps(requestedSpeedSteps, static_cast<double>(MOTOR_MAX_SPEED));
  int32_t accelSteps =
    clampSpeedSteps(requestedAccelSteps, static_cast<double>(MOTOR_MAX_ACCEL));

  m_linearMaxSpeedCmPerSec = maxSpeedCmPerSec;
  m_linearAccelCmPerSec2   = accelCmPerSec2;

  commandDrive(stepDelta, maxSpeedSteps, accelSteps, false);
}

void DifferentialDrive::turnInPlace(double degrees,
                                    double maxSpeedDegPerSec,
                                    double accelDegPerSec2) {
  commandTurn(degrees, maxSpeedDegPerSec, accelDegPerSec2, false);
}

void DifferentialDrive::moveToPose(Pose &target,
                                   double      linearSpeedCmPerSec,
                                   double      accelCmPerSec2,
                                   double      turnSpeedDegPerSec,
                                   double      turnAccelDegPerSec2) {
  m_moveTargetPose         = target;
  m_moveFinalYaw           = target.rot.getYawRads();
  m_sequenceActive         = true;
  m_mode                   = Mode::MoveToPose;
  m_subAction              = SubAction::None;
  m_movePhase              = MovePhase::RotateToHeading;
  m_linearMaxSpeedCmPerSec = linearSpeedCmPerSec;
  m_linearAccelCmPerSec2   = accelCmPerSec2;
  m_turnMaxSpeedDegPerSec  = turnSpeedDegPerSec;
  m_turnAccelDegPerSec2    = turnAccelDegPerSec2;

  double dx           = target.x - m_pose.x;
  double dy           = target.y - m_pose.y;
  m_moveTargetHeading = atan2(dy, dx);

  double yawError = normalizeAngle(m_moveTargetHeading - currentYaw());
  double distance = sqrt(dx * dx + dy * dy);

  if (distance <= kDistanceTolCm) { m_movePhase = MovePhase::FinalTurn; }

  if (fabs(yawError) > kHeadingTolRad && distance > kDistanceTolCm) {
    commandTurn(
      yawError * kRadToDeg, turnSpeedDegPerSec, turnAccelDegPerSec2, true);
  } else if (distance > kDistanceTolCm) {
    long   stepDelta = cmToSteps(distance);
    double requestedSpeedSteps =
      (linearSpeedCmPerSec > 0.0) ? linearSpeedCmPerSec * StepsPerCM : 0.0;
    double requestedAccelSteps =
      (accelCmPerSec2 > 0.0) ? accelCmPerSec2 * StepsPerCM : 0.0;
    int32_t maxSpeedSteps = clampSpeedSteps(
      requestedSpeedSteps, static_cast<double>(MOTOR_MAX_SPEED));
    int32_t accelSteps = clampSpeedSteps(requestedAccelSteps,
                                         static_cast<double>(MOTOR_MAX_ACCEL));
    commandDrive(stepDelta, maxSpeedSteps, accelSteps, true);
    m_movePhase = MovePhase::DriveStraight;
  } else {
    double finalError = normalizeAngle(m_moveFinalYaw - currentYaw());
    if (fabs(finalError) > kHeadingTolRad) {
      commandTurn(
        finalError * kRadToDeg, turnSpeedDegPerSec, turnAccelDegPerSec2, true);
      m_movePhase = MovePhase::FinalTurn;
    } else {
      m_movePhase      = MovePhase::None;
      m_sequenceActive = false;
      m_mode           = Mode::Idle;
    }
  }
}

void DifferentialDrive::stop() {
  m_left.setTarget(m_left.currentPosition());
  m_right.setTarget(m_right.currentPosition());

  m_mode           = Mode::Idle;
  m_subAction      = SubAction::None;
  m_sequenceActive = false;
  m_movePhase      = MovePhase::None;
  m_headingHold    = currentYaw();
  m_targetYaw      = m_headingHold;
}

void DifferentialDrive::update() {
  unsigned long now  = micros();
  double        dt   = (m_lastUpdateMicros > 0)
                         ? static_cast<double>(now - m_lastUpdateMicros) / 1e6
                         : 0.0;
  m_lastUpdateMicros = now;

  long leftSteps  = m_left.currentPosition();
  long rightSteps = m_right.currentPosition();

  long deltaLeft  = leftSteps - m_prevLeftSteps;
  long deltaRight = rightSteps - m_prevRightSteps;

  updateFilter(deltaLeft, deltaRight, dt);

  m_prevLeftSteps  = leftSteps;
  m_prevRightSteps = rightSteps;

  bool leftBusy  = m_left.isBusy();
  bool rightBusy = m_right.isBusy();

  if (!leftBusy && !rightBusy && !m_sequenceActive) {
    if (m_subAction == SubAction::Driving ||
        m_subAction == SubAction::Turning) {
      m_mode      = Mode::Idle;
      m_subAction = SubAction::None;
    }
  }

  if (m_subAction == SubAction::Driving) { applyHeadingCorrection(); }

  handleMoveSequence(leftBusy, rightBusy);
}

bool DifferentialDrive::isBusy() const {
  return m_left.isBusy() || m_right.isBusy();
}

void DifferentialDrive::applyHeadingCorrection() {
  if (!m_filter) return;

  double yawError     = normalizeAngle(m_headingHold - currentYaw());
  double correctionCm = std::clamp(
    yawError * (DT_TRACK_WIDTH_CM * 0.5) * kHeadingGainFactor, -5.0, 5.0);

  long correctionSteps = cnv_CMToSteps(correctionCm);

  long desiredLeft  = m_baseLeftTarget - correctionSteps;
  long desiredRight = m_baseRightTarget + correctionSteps;

  if (desiredLeft != m_left.targetPosition()) { m_left.setTarget(desiredLeft); }
  if (desiredRight != m_right.targetPosition()) {
    m_right.setTarget(desiredRight);
  }
}

void DifferentialDrive::handleMoveSequence(bool leftBusy, bool rightBusy) {
  if (!m_sequenceActive) return;

  auto finishSequence = [this]() {
    m_sequenceActive = false;
    m_movePhase      = MovePhase::None;
    m_mode           = Mode::Idle;
    m_subAction      = SubAction::None;
  };

  switch (m_movePhase) {
    case MovePhase::None: finishSequence(); break;

    case MovePhase::RotateToHeading: {
      if (m_subAction == SubAction::Turning) {
        if (!leftBusy && !rightBusy) {
          m_subAction = SubAction::None;
        } else {
          break;
        }
      }

      if (m_subAction == SubAction::None) {
        double dx       = m_moveTargetPose.x - m_pose.x;
        double dy       = m_moveTargetPose.y - m_pose.y;
        double distance = sqrt(dx * dx + dy * dy);
        if (distance <= kDistanceTolCm) {
          m_movePhase = MovePhase::FinalTurn;
        } else {
          long    stepDelta           = cmToSteps(distance);
          double  requestedSpeedSteps = (m_linearMaxSpeedCmPerSec > 0.0)
                                          ? m_linearMaxSpeedCmPerSec * StepsPerCM
                                          : 0.0;
          double  requestedAccelSteps = (m_linearAccelCmPerSec2 > 0.0)
                                          ? m_linearAccelCmPerSec2 * StepsPerCM
                                          : 0.0;
          int32_t maxSpeedSteps       = clampSpeedSteps(
            requestedSpeedSteps, static_cast<double>(MOTOR_MAX_SPEED));
          int32_t accelSteps = clampSpeedSteps(
            requestedAccelSteps, static_cast<double>(MOTOR_MAX_ACCEL));
          commandDrive(stepDelta, maxSpeedSteps, accelSteps, true);
          m_movePhase = MovePhase::DriveStraight;
        }
      }
      break;
    }

    case MovePhase::DriveStraight: {
      if (m_subAction == SubAction::Driving) {
        if (!leftBusy && !rightBusy) {
          m_subAction = SubAction::None;
        } else {
          break;
        }
      }

      if (m_subAction == SubAction::None) {
        double finalError = normalizeAngle(m_moveFinalYaw - currentYaw());
        if (fabs(finalError) <= kHeadingTolRad) {
          finishSequence();
        } else {
          commandTurn(finalError * kRadToDeg,
                      m_turnMaxSpeedDegPerSec,
                      m_turnAccelDegPerSec2,
                      true);
          m_movePhase = MovePhase::FinalTurn;
        }
      }
      break;
    }

    case MovePhase::FinalTurn: {
      if (m_subAction == SubAction::Turning) {
        if (!leftBusy && !rightBusy) {
          m_subAction = SubAction::None;
        } else {
          break;
        }
      }

      if (m_subAction == SubAction::None) {
        double finalError = normalizeAngle(m_moveFinalYaw - currentYaw());
        if (fabs(finalError) <= kHeadingTolRad) {
          finishSequence();
        } else {
          commandTurn(finalError * kRadToDeg,
                      m_turnMaxSpeedDegPerSec,
                      m_turnAccelDegPerSec2,
                      true);
        }
      }
      break;
    }
  }
}

void DifferentialDrive::updateFilter(long   deltaLeft,
                                     long   deltaRight,
                                     double dtSeconds) {
  if (!m_filter) {
    double leftCm   = cnv_stepsToCM(deltaLeft);
    double rightCm  = cnv_stepsToCM(deltaRight);
    double dCenter  = 0.5 * (leftCm + rightCm);
    double dTheta   = (rightCm - leftCm) / DT_TRACK_WIDTH_CM;
    double theta    = m_pose.rot.getYawRads();
    double thetaMid = theta + 0.5 * dTheta;

    m_pose.x += dCenter * cos(thetaMid);
    m_pose.y += dCenter * sin(thetaMid);
    m_pose.rot.setYawRads(normalizeAngle(theta + dTheta));
    return;
  }

  if (deltaLeft != 0 || deltaRight != 0 || dtSeconds > 0.0) {
    m_filter->predict(deltaLeft, deltaRight, dtSeconds);
  }

  if (m_imu && m_imu->isReady()) {
    Rotation *imuRot = m_imu->getRotation();
    if (imuRot) { m_filter->updateWithIMUYaw(imuRot->getYawRads()); }
  }

  m_pose = m_filter->getPoseEstimate();
}

double DifferentialDrive::currentYaw() const { return m_pose.rot.yaw; }

double DifferentialDrive::normalizeAngle(double angle) {
  while (angle > PI) angle -= TWO_PI;
  while (angle < -PI) angle += TWO_PI;
  return angle;
}

long DifferentialDrive::cmToSteps(double cm) { return cnv_CMToSteps(cm); }

long DifferentialDrive::degToSteps(double degrees) {
  double radians = degrees * kDegToRad;
  double arcCm   = radians * (DT_TRACK_WIDTH_CM * 0.5);
  return cnv_CMToSteps(arcCm);
}

int32_t DifferentialDrive::clampSpeedSteps(double  requested,
                                           double  defaultValue,
                                           int32_t minValue) {
  double value = (requested > 0.0) ? requested : defaultValue;
  if (value < static_cast<double>(minValue))
    value = static_cast<double>(minValue);
  int32_t steps = static_cast<int32_t>(lround(value));
  if (steps < minValue) steps = minValue;
  return steps;
}

void DifferentialDrive::commandDrive(long    stepDelta,
                                     int32_t maxSpeedSteps,
                                     int32_t accelSteps,
                                     bool    preserveSequence) {
  if (!preserveSequence) {
    m_sequenceActive = false;
    m_movePhase      = MovePhase::None;
  }

  m_mode        = preserveSequence ? Mode::MoveToPose : Mode::DrivingStraight;
  m_subAction   = SubAction::Driving;
  m_headingHold = currentYaw();
  m_targetYaw   = m_headingHold;

  m_baseLeftTarget  = m_left.currentPosition() + stepDelta;
  m_baseRightTarget = m_right.currentPosition() + stepDelta;

  m_left.moveBy(stepDelta, maxSpeedSteps, accelSteps);
  m_right.moveBy(stepDelta, maxSpeedSteps, accelSteps);
}

void DifferentialDrive::commandTurn(double degrees,
                                    double maxSpeedDegPerSec,
                                    double accelDegPerSec2,
                                    bool   preserveSequence) {
  double radians = degrees * kDegToRad;
  if (fabs(radians) < 1e-6) return;

  if (!preserveSequence) {
    m_sequenceActive = false;
    m_movePhase      = MovePhase::None;
  }

  m_mode        = preserveSequence ? Mode::MoveToPose : Mode::Turning;
  m_subAction   = SubAction::Turning;
  m_headingHold = currentYaw();
  m_targetYaw   = normalizeAngle(m_headingHold + radians);

  double arcCm     = radians * (DT_TRACK_WIDTH_CM * 0.5);
  long   stepDelta = cnv_CMToSteps(arcCm);

  double requestedSpeedSteps = (maxSpeedDegPerSec > 0.0)
                                 ? fabs(maxSpeedDegPerSec) * kDegToRad *
                                     (DT_TRACK_WIDTH_CM * 0.5) * StepsPerCM
                                 : 0.0;
  double requestedAccelSteps = (accelDegPerSec2 > 0.0)
                                 ? fabs(accelDegPerSec2) * kDegToRad *
                                     (DT_TRACK_WIDTH_CM * 0.5) * StepsPerCM
                                 : 0.0;

  int32_t maxSpeedSteps =
    clampSpeedSteps(requestedSpeedSteps, static_cast<double>(MOTOR_MAX_SPEED));
  int32_t accelSteps =
    clampSpeedSteps(requestedAccelSteps, static_cast<double>(MOTOR_MAX_ACCEL));

  m_baseLeftTarget  = m_left.currentPosition() - stepDelta;
  m_baseRightTarget = m_right.currentPosition() + stepDelta;

  m_left.moveBy(-stepDelta, maxSpeedSteps, accelSteps);
  m_right.moveBy(stepDelta, maxSpeedSteps, accelSteps);
}
