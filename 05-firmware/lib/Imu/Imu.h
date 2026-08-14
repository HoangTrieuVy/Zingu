// MPU-6050 driver plus a complementary filter that fuses accelerometer tilt
// with integrated gyro rate into a single pitch estimate.
//
// Only the pitch axis is fused. Zingu balances on one axis; yaw and roll are
// read but left unfiltered, since nothing in the control loop consumes them.

#pragma once

#include <stdint.h>

class Imu {
 public:
  // Brings up I2C and wakes the MPU-6050. Returns false if the device does not
  // answer with the expected WHO_AM_I value.
  bool begin();

  // Averages gyro samples to find the zero-rate bias. The robot must be still
  // and flat. Returns false if it moved during the sample window, in which case
  // the previous bias is left untouched.
  bool calibrateGyro();

  // Reads the sensor and advances the filter by dt seconds. Call this exactly
  // once per control tick.
  void update(float dt);

  // Degrees from vertical, corrected by cfg::kPitchOffsetDeg. Positive is
  // forward (nose-down) lean.
  float pitchDeg() const { return pitch_deg_; }

  // Degrees per second about the pitch axis, gyro bias removed.
  float pitchRateDegPerSec() const { return pitch_rate_dps_; }

  // Raw accelerometer magnitude in g. Near 1.0 when at rest; a sustained
  // departure means the chassis is being shaken, carried, or has been dropped.
  float accelMagnitudeG() const { return accel_magnitude_g_; }

  // False once a read has failed, which the caller should treat as a fault:
  // a balancer with no angle estimate must not drive its motors.
  bool healthy() const { return healthy_; }

 private:
  struct Sample {
    float ax, ay, az;  // g
    float gx, gy, gz;  // deg/s
  };

  bool readRaw(Sample& out);

  float pitch_deg_ = 0.0f;
  float pitch_rate_dps_ = 0.0f;
  float accel_magnitude_g_ = 1.0f;
  float gyro_bias_dps_ = 0.0f;
  bool filter_seeded_ = false;
  bool healthy_ = false;
};
