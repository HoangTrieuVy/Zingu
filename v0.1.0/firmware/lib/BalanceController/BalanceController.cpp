#include "BalanceController.h"

#include <math.h>

#include "config.h"

namespace {

float clamp(float value, float limit) {
  if (value > limit) return limit;
  if (value < -limit) return -limit;
  return value;
}

}  // namespace

void BalanceController::reset() {
  velocity_integral_ = 0.0f;
  pitch_integral_ = 0.0f;
  last_pitch_error_deg_ = 0.0f;
  pitch_setpoint_deg_ = 0.0f;
  has_last_error_ = false;
}

float BalanceController::update(float pitch_deg, float pitch_rate_dps,
                                float wheel_velocity_rps,
                                float target_velocity_rps, float dt) {
  // --- Outer loop: velocity error -> pitch setpoint --------------------------
  const float velocity_error = target_velocity_rps - wheel_velocity_rps;
  velocity_integral_ =
      clamp(velocity_integral_ + velocity_error * dt, cfg::kIntegratorLimit);

  // Negated: to gain forward speed the robot must lean forward, which is a
  // negative pitch error for the inner loop to chase.
  pitch_setpoint_deg_ =
      clamp(-(cfg::kVelocityKp * velocity_error +
              cfg::kVelocityKi * velocity_integral_),
            cfg::kVelocityPitchLimitDeg);

  // --- Inner loop: pitch error -> duty ---------------------------------------
  const float pitch_error = pitch_setpoint_deg_ - pitch_deg;

  if (!has_last_error_) {
    last_pitch_error_deg_ = pitch_error;
    has_last_error_ = true;
  }

  pitch_integral_ =
      clamp(pitch_integral_ + pitch_error * dt, cfg::kIntegratorLimit);

  // Derivative on the gyro rate, not on the error difference. The gyro is a
  // direct measurement of d(pitch)/dt, so it carries none of the quantisation
  // noise that differencing the filtered angle would amplify.
  const float derivative = -pitch_rate_dps;

  const float duty = cfg::kPitchKp * pitch_error +
                     cfg::kPitchKi * pitch_integral_ +
                     cfg::kPitchKd * derivative;

  last_pitch_error_deg_ = pitch_error;

  // Saturating here rather than letting the caller clip keeps the integrator
  // clamp above the only anti-windup mechanism, so its behaviour is predictable.
  if (duty > 1.0f) return 1.0f;
  if (duty < -1.0f) return -1.0f;
  return duty;
}
