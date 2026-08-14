#include "Imu.h"

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "config.h"

namespace {

constexpr uint8_t kRegSmplrtDiv = 0x19;
constexpr uint8_t kRegConfig = 0x1A;
constexpr uint8_t kRegGyroConfig = 0x1B;
constexpr uint8_t kRegAccelConfig = 0x1C;
constexpr uint8_t kRegAccelXoutH = 0x3B;
constexpr uint8_t kRegPwrMgmt1 = 0x6B;
constexpr uint8_t kRegWhoAmI = 0x75;

// +/-4 g and +/-500 deg/s. Wide enough that the recovery kick does not clip the
// gyro, tight enough to keep useful resolution while balancing.
constexpr uint8_t kAccelFsSel4G = 0x08;
constexpr uint8_t kGyroFsSel500Dps = 0x08;
constexpr float kAccelScaleLsbPerG = 8192.0f;
constexpr float kGyroScaleLsbPerDps = 65.5f;

bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(cfg::kMpu6050Address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readRegisters(uint8_t reg, uint8_t* buffer, uint8_t length) {
  Wire.beginTransmission(cfg::kMpu6050Address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(cfg::kMpu6050Address, length) != length) {
    return false;
  }
  for (uint8_t i = 0; i < length; ++i) {
    buffer[i] = Wire.read();
  }
  return true;
}

int16_t combine(uint8_t high, uint8_t low) {
  return static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
}

}  // namespace

bool Imu::begin() {
  Wire.begin(cfg::kI2cSdaPin, cfg::kI2cSclPin, cfg::kI2cClockHz);

  uint8_t who = 0;
  if (!readRegisters(kRegWhoAmI, &who, 1) || who != cfg::kMpu6050Address) {
    healthy_ = false;
    return false;
  }

  // Wake from sleep and clock off the X gyro PLL, which is more stable than the
  // internal oscillator.
  const bool ok = writeRegister(kRegPwrMgmt1, 0x01) &&
                  writeRegister(kRegSmplrtDiv, 0x00) &&
                  // DLPF 44 Hz: rolls off chassis vibration without adding so
                  // much phase lag that the D term goes useless.
                  writeRegister(kRegConfig, 0x03) &&
                  writeRegister(kRegGyroConfig, kGyroFsSel500Dps) &&
                  writeRegister(kRegAccelConfig, kAccelFsSel4G);

  healthy_ = ok;
  return ok;
}

bool Imu::readRaw(Sample& out) {
  uint8_t buf[14];
  if (!readRegisters(kRegAccelXoutH, buf, sizeof(buf))) {
    return false;
  }

  out.ax = combine(buf[0], buf[1]) / kAccelScaleLsbPerG;
  out.ay = combine(buf[2], buf[3]) / kAccelScaleLsbPerG;
  out.az = combine(buf[4], buf[5]) / kAccelScaleLsbPerG;
  // buf[6..7] is temperature, unused.
  out.gx = combine(buf[8], buf[9]) / kGyroScaleLsbPerDps;
  out.gy = combine(buf[10], buf[11]) / kGyroScaleLsbPerDps;
  out.gz = combine(buf[12], buf[13]) / kGyroScaleLsbPerDps;
  return true;
}

bool Imu::calibrateGyro() {
  double sum = 0.0;
  float min_rate = 1e9f;
  float max_rate = -1e9f;

  for (uint32_t i = 0; i < cfg::kGyroCalibrationSamples; ++i) {
    Sample s;
    if (!readRaw(s)) {
      healthy_ = false;
      return false;
    }
    sum += s.gy;
    min_rate = fminf(min_rate, s.gy);
    max_rate = fmaxf(max_rate, s.gy);
    delayMicroseconds(1000);
  }

  // A spread this wide means the robot was not actually still, so the average
  // is not a bias — it is motion, and baking it in would make the robot drift.
  if ((max_rate - min_rate) > cfg::kCalibrationMaxRateDegPerSec * 2.0f) {
    return false;
  }

  gyro_bias_dps_ = static_cast<float>(sum / cfg::kGyroCalibrationSamples);
  filter_seeded_ = false;
  return true;
}

void Imu::update(float dt) {
  Sample s;
  if (!readRaw(s)) {
    healthy_ = false;
    return;
  }
  healthy_ = true;

  accel_magnitude_g_ = sqrtf(s.ax * s.ax + s.ay * s.ay + s.az * s.az);

  // Tilt from gravity. atan2 over the full circle rather than asin, so the
  // estimate stays meaningful while the robot is lying down during recovery.
  const float accel_pitch_deg =
      atan2f(-s.ax, sqrtf(s.ay * s.ay + s.az * s.az)) * 180.0f / PI;

  pitch_rate_dps_ = s.gy - gyro_bias_dps_;

  if (!filter_seeded_) {
    // Start on the accelerometer so the filter does not have to converge from
    // zero while the robot is already leaning.
    pitch_deg_ = accel_pitch_deg - cfg::kPitchOffsetDeg;
    filter_seeded_ = true;
    return;
  }

  const float gyro_pitch_deg = pitch_deg_ + pitch_rate_dps_ * dt;
  pitch_deg_ = cfg::kComplementaryAlpha * gyro_pitch_deg +
               (1.0f - cfg::kComplementaryAlpha) *
                   (accel_pitch_deg - cfg::kPitchOffsetDeg);
}
