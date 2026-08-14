// The five states Zingu can be in, and the names used for them in telemetry.

#pragma once

#include <stdint.h>

enum class RobotState : uint8_t {
  // Disarmed. Motors off, IMU streaming. Entered at boot and after a fault
  // clears.
  kIdle,

  // On the ground, running the kick-up maneuver.
  kRecovering,

  // Upright and under closed-loop control.
  kBalancing,

  // Something is wrong that the robot cannot drive its way out of: IMU dead, or
  // the recovery attempt budget spent. Motors stay off until power-cycled.
  kFault,
};

inline const char* toString(RobotState state) {
  switch (state) {
    case RobotState::kIdle: return "IDLE";
    case RobotState::kRecovering: return "RECOVERING";
    case RobotState::kBalancing: return "BALANCING";
    case RobotState::kFault: return "FAULT";
  }
  return "UNKNOWN";
}
