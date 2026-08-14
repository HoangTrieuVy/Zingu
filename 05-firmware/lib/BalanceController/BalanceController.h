// Cascaded PID that keeps the chassis upright.
//
//   wheel velocity error --> [ PI ] --> pitch setpoint --> [ PID ] --> duty
//
// The outer loop is the counter-intuitive half. To slow a runaway forward roll
// the robot must first lean *further* forward, drive the wheels under the centre
// of mass, and only then straighten up. So velocity error is converted into a
// pitch target rather than directly into motor command.

#pragma once

class BalanceController {
 public:
  // Clears both integrators and the derivative memory. Call on every entry to
  // BALANCING: stale integral from before a fall would otherwise be dumped into
  // the motors the instant the robot is caught.
  void reset();

  // Runs one control tick and returns signed drive duty in [-1, 1].
  //   pitch_deg           current lean, positive forward
  //   pitch_rate_dps      gyro rate about the pitch axis
  //   wheel_velocity_rps  measured mean wheel velocity
  //   target_velocity_rps commanded travel speed; 0 holds station
  float update(float pitch_deg, float pitch_rate_dps, float wheel_velocity_rps,
               float target_velocity_rps, float dt);

  // Pitch setpoint the outer loop asked for on the last tick. Telemetry only.
  float pitchSetpointDeg() const { return pitch_setpoint_deg_; }

 private:
  float velocity_integral_ = 0.0f;
  float pitch_integral_ = 0.0f;
  float last_pitch_error_deg_ = 0.0f;
  float pitch_setpoint_deg_ = 0.0f;
  bool has_last_error_ = false;
};
