#include "RecoveryManeuver.h"

#include <math.h>

#include "MotorDriver.h"
#include "config.h"

void RecoveryManeuver::reset() {
  phase_ = Phase::kDone;
  attempts_ = 0;
  settled_since_ms_ = 0;
}

void RecoveryManeuver::begin(bool fell_forward) {
  fell_forward_ = fell_forward;
  phase_ = Phase::kSettle;
  phase_started_ms_ = 0;
  settled_since_ms_ = 0;
}

void RecoveryManeuver::enter(Phase phase, uint32_t now_ms) {
  phase_ = phase;
  phase_started_ms_ = now_ms;
}

void RecoveryManeuver::retry(uint32_t now_ms) {
  ++attempts_;
  if (attempts_ >= cfg::kRecoveryMaxAttempts) {
    phase_ = Phase::kExhausted;
    return;
  }
  // Back to settle: the failed attempt almost certainly left the chassis
  // rocking, and kicking into a moving body is how you get a sideways launch.
  phase_ = Phase::kSettle;
  settled_since_ms_ = 0;
  phase_started_ms_ = now_ms + cfg::kRecoveryRetryDelayMs;
}

void RecoveryManeuver::update(MotorDriver& motors, float pitch_rate_dps,
                              uint32_t now_ms) {
  // Sign convention: a forward fall needs the wheels driven backward to swing
  // the body back over them, and vice versa.
  const float kick_sign = fell_forward_ ? -1.0f : 1.0f;

  switch (phase_) {
    case Phase::kSettle: {
      motors.coast();

      // Honour the post-failure delay before even looking at the gyro.
      if (now_ms < phase_started_ms_) {
        settled_since_ms_ = 0;
        return;
      }

      if (fabsf(pitch_rate_dps) > cfg::kSettledRateDegPerSec) {
        settled_since_ms_ = 0;  // still moving; restart the hold timer
        return;
      }
      if (settled_since_ms_ == 0) {
        settled_since_ms_ = now_ms;
        return;
      }
      if (now_ms - settled_since_ms_ >= cfg::kSettleHoldMs) {
        motors.resetOdometry();
        enter(Phase::kWindup, now_ms);
      }
      return;
    }

    case Phase::kWindup: {
      // Drive *against* the kick direction. This rocks the chassis up onto the
      // edge it will pivot over, so the kick starts with the body already
      // moving the right way instead of fighting static friction.
      const float duty = -kick_sign * cfg::kRecoveryWindupDuty;
      motors.setDuty(duty, duty);
      if (now_ms - phase_started_ms_ >= cfg::kRecoveryWindupMs) {
        enter(Phase::kKick, now_ms);
      }
      return;
    }

    case Phase::kKick: {
      const float duty = kick_sign * cfg::kRecoveryKickDuty;
      motors.setDuty(duty, duty);
      if (now_ms - phase_started_ms_ >= cfg::kRecoveryKickMs) {
        enter(Phase::kCoast, now_ms);
      }
      return;
    }

    case Phase::kCoast: {
      // Free the wheels so the chassis can rotate the rest of the way on its own
      // momentum. Braking here would plant the wheels and stop the swing short.
      motors.coast();
      if (now_ms - phase_started_ms_ >= cfg::kRecoveryCoastMs) {
        enter(Phase::kDone, now_ms);
      }
      return;
    }

    case Phase::kDone:
    case Phase::kExhausted:
      motors.coast();
      return;
  }
}
