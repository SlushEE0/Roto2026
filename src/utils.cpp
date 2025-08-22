#include <math.h>
#include <BasicLinearAlgebra.h>

#include <utils.h>

static inline long cnv_stepsToMM(long steps) {
  return steps * MMsPerStep;
}

static inline long cnv_MMToSteps(long mm) {
  return mm / MMsPerStep;
}

RotationEuler Rotation::getEuler() {
  return cnv_quatToEuler(&rotation);
}

RotationQuat Rotation::getQuat() {
  return rotation;
}

void Rotation::setEuler(RotationEuler *euler) {
  rotation = cnv_eulerToQuat(euler);
}

void Rotation::setQuat(RotationQuat *quat) {
  rotation = *quat;
}

RotationEuler cnv_quatToEuler(RotationQuat *ptr_r) {
  RotationQuat r = *ptr_r;
  RotationEuler e;

  // Normalize quaternion to prevent numerical errors
  double norm = sqrt(r.w * r.w + r.i * r.i + r.j * r.j + r.k * r.k);
  if (norm > 0.0) {
    r.w /= norm;
    r.i /= norm;
    r.j /= norm;
    r.k /= norm;
  }

  // Calculate Euler angles with proper clamping
  double roll =
    atan2(2.0 * (r.w * r.i + r.j * r.k), 1.0 - 2.0 * (r.i * r.i + r.j * r.j));

  // Clamp the input to asin to prevent NaN
  double pitch_input = 2.0 * (r.w * r.j - r.k * r.i);
  if (pitch_input > 1.0)
    pitch_input = 1.0;
  if (pitch_input < -1.0)
    pitch_input = -1.0;
  double pitch = asin(pitch_input);

  double yaw =
    atan2(2.0 * (r.w * r.k + r.i * r.j), 1.0 - 2.0 * (r.j * r.j + r.k * r.k));

  // Convert to degrees
  e.x = roll * 180.0 / PI;
  e.y = pitch * 180.0 / PI;
  e.z = yaw * 180.0 / PI;

  // Convert from -180 to 180 range to 0 to 360 range
  if (e.x < 0.0f)
    e.x += 360.0f;
  if (e.y < 0.0f)
    e.y += 360.0f;
  if (e.z < 0.0f)
    e.z += 360.0f;

  return e;
}

RotationQuat cnv_eulerToQuat(RotationEuler *ptr_r) {
  RotationEuler r = *ptr_r;
  RotationQuat result;

  // Convert degrees to radians and normalize to 0-360 range
  double roll_rad = fmod(r.x + 720.0, 360.0) * PI / 180.0;
  double pitch_rad = fmod(r.y + 720.0, 360.0) * PI / 180.0;
  double yaw_rad = fmod(r.z + 720.0, 360.0) * PI / 180.0;

  // Calculate half angles
  double cr = cos(roll_rad * 0.5);
  double sr = sin(roll_rad * 0.5);
  double cp = cos(pitch_rad * 0.5);
  double sp = sin(pitch_rad * 0.5);
  double cy = cos(yaw_rad * 0.5);
  double sy = sin(yaw_rad * 0.5);

  // Calculate quaternion components
  result.w = cr * cp * cy + sr * sp * sy;
  result.i = sr * cp * cy - cr * sp * sy;
  result.j = cr * sp * cy + sr * cp * sy;
  result.k = cr * cp * sy - sr * sp * cy;

  return result;
};

void normalizeQuat(RotationQuat &q) {
  double norm = sqrt(q.w * q.w + q.i * q.i + q.j * q.j + q.k * q.k);
  if (norm > 0.0) {
    q.w = q.w / norm;
    q.i = q.i / norm;
    q.j = q.j / norm;
    q.k = q.k / norm;
  }
}

bool clamp(long *value, long min, long max) {
  if (*value < min) {
    *value = min;
  } else if (*value > max) {
    *value = max;
  } else {
    return false;
  }

  return true;
}
