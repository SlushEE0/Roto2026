#include <math.h>

#include <utils.h>

static inline long cnv_stepsToMM(long steps) {
  return steps * MMsPerStep;
}

static inline long cnv_MMToSteps(long mm) {
  return mm / MMsPerStep;
}

RotationEuler cnv_quatToEuler(RotationQuat q) {
  RotationEuler e;

  // Normalize quaternion to prevent numerical errors
  double norm = sqrt(q.w * q.w + q.i * q.i + q.j * q.j + q.k * q.k);
  if (norm > 0.0) {
    q.w /= norm;
    q.i /= norm;
    q.j /= norm;
    q.k /= norm;
  }

  // Calculate Euler angles with proper clamping
  double roll =
    atan2(2.0 * (q.w * q.i + q.j * q.k), 1.0 - 2.0 * (q.i * q.i + q.j * q.j));

  // Clamp the input to asin to prevent NaN
  double pitch_input = 2.0 * (q.w * q.j - q.k * q.i);
  if (pitch_input > 1.0)
    pitch_input = 1.0;
  if (pitch_input < -1.0)
    pitch_input = -1.0;
  double pitch = asin(pitch_input);

  double yaw =
    atan2(2.0 * (q.w * q.k + q.i * q.j), 1.0 - 2.0 * (q.j * q.j + q.k * q.k));

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
