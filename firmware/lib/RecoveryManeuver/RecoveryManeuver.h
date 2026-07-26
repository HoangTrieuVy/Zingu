// Open-loop kick-up that returns a fallen chassis to the balance envelope.
//
// Physics: the wheels cannot lift the body directly, but the reaction torque of
// accelerating them can. Spinning the wheels one way drives the chassis to
// rotate the other. The maneuver builds angular momentum against the ground
// (WINDUP), then reverses hard (KICK) so the body whips through vertical, where
// the balance controller takes over.
//
//   SETTLE -> WINDUP -> KICK -> COAST -> (caught? balance : retry)
//
// Phase transitions are timed, not sensed: the whole move lasts a few hundred
// milliseconds and the IMU estimate is unreliable mid-flip, so closing the loop
// on angle here would do more harm than good. Success is judged only at the end.

#pragma once

#include <stdint.h>

class MotorDriver;

class RecoveryManeuver {
 public:
  enum class Phase : uint8_t {
    kSettle,   // waiting for the chassis to stop bouncing
    kWindup,   // torque away from the fall direction
    kKick,     // full reverse torque; the body swings up
    kCoast,    // wheels free, letting the chassis arrive
    kDone,     // maneuver finished; caller checks whether it worked
    kExhausted // attempt budget spent
  };

  // Starts a recovery sequence. fell_forward selects which way the kick points.
  void begin(bool fell_forward);

  // Advances the maneuver and commands the motors. Call once per control tick
  // with the current gyro rate, which is used only to decide when the chassis
  // has settled enough to start.
  void update(MotorDriver& motors, float pitch_rate_dps, uint32_t now_ms);

  Phase phase() const { return phase_; }
  bool finished() const {
    return phase_ == Phase::kDone || phase_ == Phase::kExhausted;
  }
  uint8_t attempts() const { return attempts_; }

  // Called when a completed attempt failed to land the robot upright. Queues
  // another try, or gives up once the budget is spent.
  void retry(uint32_t now_ms);

  // Called on a successful catch, so the next fall starts from a clean budget.
  void reset();

 private:
  void enter(Phase phase, uint32_t now_ms);

  Phase phase_ = Phase::kDone;
  uint32_t phase_started_ms_ = 0;
  uint32_t settled_since_ms_ = 0;
  bool fell_forward_ = true;
  uint8_t attempts_ = 0;
};
