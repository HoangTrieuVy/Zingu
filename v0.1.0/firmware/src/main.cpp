// Zingu — self-balancing, self-recovering two-wheeled robot.
//
// One fixed-rate control loop drives a four-state machine. Every tick reads the
// IMU and encoders, then dispatches on state; only BALANCING and RECOVERING
// ever command the motors.

#include <Arduino.h>

#include "BalanceController.h"
#include "Imu.h"
#include "MotorDriver.h"
#include "RecoveryManeuver.h"
#include "RobotState.h"
#include "config.h"

namespace {

Imu g_imu;
MotorDriver g_motors;
BalanceController g_balance;
RecoveryManeuver g_recovery;

RobotState g_state = RobotState::kIdle;
uint32_t g_next_tick_us = 0;
uint32_t g_last_telemetry_ms = 0;

// Commanded travel speed. Zero means hold station; a future radio or serial
// command layer writes this.
float g_target_velocity_rps = 0.0f;

void transitionTo(RobotState next, uint32_t now_ms) {
  if (next == g_state) {
    return;
  }

  switch (next) {
    case RobotState::kBalancing:
      // Fresh integrators, fresh odometry: whatever the wheels did during the
      // fall and the kick is not error the balance loop should answer for.
      g_balance.reset();
      g_motors.resetOdometry();
      g_recovery.reset();
      break;

    case RobotState::kRecovering:
      // Pitch sign at the moment of the fall tells us which edge we are lying
      // on, and therefore which way to kick.
      g_recovery.begin(g_imu.pitchDeg() > 0.0f);
      break;

    case RobotState::kIdle:
    case RobotState::kFault:
      g_motors.coast();
      break;
  }

  Serial.printf("[state] %s -> %s @ %lu ms\n", toString(g_state),
                toString(next), static_cast<unsigned long>(now_ms));
  g_state = next;
}

bool withinUprightEnvelope() {
  return fabsf(g_imu.pitchDeg()) < cfg::kUprightPitchDeg &&
         fabsf(g_imu.pitchRateDegPerSec()) < cfg::kUprightRateDegPerSec;
}

void runIdle(uint32_t now_ms) {
  g_motors.coast();

  // Arm as soon as somebody stands the robot up by hand.
  if (withinUprightEnvelope()) {
    transitionTo(RobotState::kBalancing, now_ms);
  }
}

void runBalancing(uint32_t now_ms) {
  if (fabsf(g_imu.pitchDeg()) > cfg::kFallenPitchDeg) {
    transitionTo(RobotState::kRecovering, now_ms);
    return;
  }

  const float duty =
      g_balance.update(g_imu.pitchDeg(), g_imu.pitchRateDegPerSec(),
                       g_motors.wheelVelocityRps(), g_target_velocity_rps,
                       cfg::kControlDt);
  g_motors.setDuty(duty, duty);
}

void runRecovering(uint32_t now_ms) {
  g_recovery.update(g_motors, g_imu.pitchRateDegPerSec(), now_ms);

  if (g_recovery.phase() == RecoveryManeuver::Phase::kExhausted) {
    Serial.println("[recovery] attempt budget exhausted");
    transitionTo(RobotState::kFault, now_ms);
    return;
  }

  if (!g_recovery.finished()) {
    return;
  }

  // The maneuver has played out. Did it land?
  if (withinUprightEnvelope()) {
    Serial.printf("[recovery] caught after %u attempt(s)\n",
                  g_recovery.attempts() + 1);
    transitionTo(RobotState::kBalancing, now_ms);
  } else {
    g_recovery.retry(now_ms);
  }
}

void emitTelemetry(uint32_t now_ms) {
  if (now_ms - g_last_telemetry_ms < 1000 / cfg::kTelemetryHz) {
    return;
  }
  g_last_telemetry_ms = now_ms;

  // CSV, consumed by tools/telemetry_plot.py.
  Serial.printf("T,%lu,%s,%.2f,%.2f,%.2f,%.3f,%.2f\n",
                static_cast<unsigned long>(now_ms), toString(g_state),
                g_imu.pitchDeg(), g_balance.pitchSetpointDeg(),
                g_imu.pitchRateDegPerSec(), g_motors.wheelVelocityRps(),
                g_imu.accelMagnitudeG());
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== Zingu ===");

  g_motors.begin();

  if (!g_imu.begin()) {
    Serial.println("[fault] MPU-6050 not responding");
    g_state = RobotState::kFault;
    return;
  }

  Serial.println("[boot] hold still, calibrating gyro...");
  if (!g_imu.calibrateGyro()) {
    Serial.println("[fault] gyro calibration failed — robot was moving");
    g_state = RobotState::kFault;
    return;
  }
  Serial.println("[boot] ready — stand the robot up to arm");

  g_motors.resetOdometry();
  g_next_tick_us = micros();
}

void loop() {
  // Fixed-rate scheduler. Everything downstream assumes cfg::kControlDt, so the
  // loop period has to be held rather than left to run free.
  const uint32_t now_us = micros();
  if (static_cast<int32_t>(now_us - g_next_tick_us) < 0) {
    return;
  }
  g_next_tick_us += cfg::kControlPeriodUs;

  const uint32_t now_ms = millis();

  g_imu.update(cfg::kControlDt);
  g_motors.update(cfg::kControlDt);

  // A dead IMU is unrecoverable in software: with no angle estimate, driving
  // the motors is strictly worse than stopping.
  if (!g_imu.healthy() && g_state != RobotState::kFault) {
    Serial.println("[fault] lost the IMU");
    transitionTo(RobotState::kFault, now_ms);
  }

  switch (g_state) {
    case RobotState::kIdle:       runIdle(now_ms); break;
    case RobotState::kBalancing:  runBalancing(now_ms); break;
    case RobotState::kRecovering: runRecovering(now_ms); break;
    case RobotState::kFault:      g_motors.coast(); break;
  }

  emitTelemetry(now_ms);
}
