// Zingu — single place for every hardware pin, timing constant and control gain.
//
// Everything here is chassis-specific. The gains below are conservative
// placeholders, NOT tuned values. Read docs/tuning.md before the robot is
// allowed to move under its own power.

#pragma once

#include <stdint.h>

namespace cfg {

// ---------------------------------------------------------------- I2C / IMU --
constexpr uint8_t kI2cSdaPin = 21;
constexpr uint8_t kI2cSclPin = 22;
constexpr uint32_t kI2cClockHz = 400000;
constexpr uint8_t kMpu6050Address = 0x68;

// Mounting correction: pitch reported by the IMU when the chassis is balanced
// on its wheels. Measure this once, with the robot held upright by hand.
constexpr float kPitchOffsetDeg = 0.0f;

// Complementary filter weight on the gyro-integrated angle. Higher = trusts the
// gyro more and the accelerometer less; 0.98 at 200 Hz is a ~2.5 s time constant.
constexpr float kComplementaryAlpha = 0.98f;

// -------------------------------------------------------------- Motor pins --
// TB6612FNG dual H-bridge.
constexpr uint8_t kMotorLeftPwmPin = 25;
constexpr uint8_t kMotorLeftIn1Pin = 26;
constexpr uint8_t kMotorLeftIn2Pin = 27;
constexpr uint8_t kMotorRightPwmPin = 32;
constexpr uint8_t kMotorRightIn1Pin = 33;
constexpr uint8_t kMotorRightIn2Pin = 14;
constexpr uint8_t kMotorStandbyPin = 13;

constexpr uint32_t kMotorPwmFrequencyHz = 20000;  // above audible range
constexpr uint8_t kMotorPwmResolutionBits = 10;   // duty 0..1023

// Duty below which the gearmotors merely buzz instead of turning. The driver
// snaps small commands up to this so the controller sees no dead zone.
constexpr float kMotorDeadbandDuty = 0.08f;

// --------------------------------------------------------------- Encoders ---
constexpr uint8_t kEncoderLeftAPin = 34;
constexpr uint8_t kEncoderLeftBPin = 35;
constexpr uint8_t kEncoderRightAPin = 36;
constexpr uint8_t kEncoderRightBPin = 39;
constexpr float kEncoderCountsPerWheelRev = 1200.0f;  // 4x quadrature x gearbox

// ------------------------------------------------------------ Loop timing ---
constexpr uint32_t kControlLoopHz = 200;
constexpr uint32_t kControlPeriodUs = 1000000UL / kControlLoopHz;
constexpr float kControlDt = 1.0f / static_cast<float>(kControlLoopHz);

constexpr uint32_t kTelemetryHz = 20;

// -------------------------------------------------------- Balance control ---
// Outer loop: wheel velocity error -> pitch setpoint.
// Leaning into the error is what makes the robot chase its own base back under
// its centre of mass, so this loop is deliberately slow and gently limited.
constexpr float kVelocityKp = 0.030f;
constexpr float kVelocityKi = 0.010f;
constexpr float kVelocityPitchLimitDeg = 6.0f;

// Inner loop: pitch error -> motor duty.
constexpr float kPitchKp = 0.040f;
constexpr float kPitchKi = 0.150f;
constexpr float kPitchKd = 0.0012f;

// Integrator clamp, in duty units, shared by both loops.
constexpr float kIntegratorLimit = 0.60f;

// ---------------------------------------------------------- State machine ---
// Balancing is abandoned beyond this tilt — past it the wheels cannot generate
// enough horizontal force to recover the centre of mass.
constexpr float kFallenPitchDeg = 42.0f;

// Re-entry envelope. Tighter than the fall threshold on purpose: the recovery
// kick must deliver the chassis genuinely near upright, not merely inside the
// old limit, or the controller will immediately lose it again.
constexpr float kUprightPitchDeg = 12.0f;
constexpr float kUprightRateDegPerSec = 90.0f;

// The chassis must stop bouncing before a kick-up is attempted.
constexpr float kSettledRateDegPerSec = 20.0f;
constexpr uint32_t kSettleHoldMs = 400;

// ------------------------------------------------------ Recovery maneuver ---
// Open-loop, two-phase kick-up. Timings are chassis mass/geometry dependent.
constexpr float kRecoveryWindupDuty = 0.55f;
constexpr uint32_t kRecoveryWindupMs = 260;
constexpr float kRecoveryKickDuty = 1.00f;
constexpr uint32_t kRecoveryKickMs = 180;
constexpr uint32_t kRecoveryCoastMs = 120;

// Give up and fault after this many consecutive failed attempts, so a robot
// wedged against a wall stops hammering its own gearboxes.
constexpr uint8_t kRecoveryMaxAttempts = 4;
constexpr uint32_t kRecoveryRetryDelayMs = 800;

// ------------------------------------------------------------ Calibration ---
constexpr uint32_t kGyroCalibrationSamples = 1000;
constexpr float kCalibrationMaxRateDegPerSec = 2.0f;  // reject if it moved

}  // namespace cfg
