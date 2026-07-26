// Two brushed gearmotors behind a TB6612FNG, plus their quadrature encoders.
//
// The controller above this layer thinks in signed duty (-1..+1) and in wheel
// velocity (revolutions per second). Nothing above here knows about PWM
// channels, direction pins, or encoder counts.

#pragma once

#include <stdint.h>

class MotorDriver {
 public:
  void begin();

  // Signed duty per side, clamped to [-1, 1]. Values inside the mechanical
  // deadband are pushed out to the smallest duty that actually turns the wheel,
  // so the control loop does not have to model a flat spot around zero.
  void setDuty(float left, float right);

  // Cuts drive and lets the wheels spin down. Used between recovery phases and
  // whenever the robot is disarmed.
  void coast();

  // Shorts the motor terminals for active braking. Sharper than coast(); used
  // to stop the wind-up dead before the kick reverses.
  void brake();

  // Samples the encoders and updates the velocity estimate. Call once per
  // control tick, before reading velocity.
  void update(float dt);

  // Mean of both wheels, revolutions per second. Positive drives the robot
  // forward. This is what the outer velocity loop regulates.
  float wheelVelocityRps() const { return wheel_velocity_rps_; }

  // Signed wheel travel in revolutions since the last resetOdometry().
  float wheelPositionRev() const { return wheel_position_rev_; }

  void resetOdometry();

 private:
  void applySide(uint8_t pwm_channel, uint8_t in1_pin, uint8_t in2_pin,
                 float duty);

  float wheel_velocity_rps_ = 0.0f;
  float wheel_position_rev_ = 0.0f;
  int32_t last_left_count_ = 0;
  int32_t last_right_count_ = 0;
};
