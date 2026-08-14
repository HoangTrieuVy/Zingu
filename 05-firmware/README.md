# Stage 5 — Firmware

**Goal:** ESP32 firmware that balances, recovers from falls, and — critically for what
follows — streams timestamped telemetry good enough to identify the robot from.

**Prerequisites:** assembled hardware from stage 4, PlatformIO installed.

## Quick start

```bash
pip install platformio

cd 05-firmware
pio run                 # build
pio run -t upload       # flash the ESP32
pio device monitor      # watch telemetry at 115200 baud
```

On first boot the robot stays in `IDLE` and streams IMU readings. Lay it flat and leave
it still for five seconds — it calibrates the gyro bias, then arms itself.

## Layout

| Path | Contents |
| ---- | -------- |
| `src/main.cpp` | Scheduler and top-level state machine |
| `lib/Imu/` | MPU-6050 driver, bias calibration, complementary filter |
| `lib/MotorDriver/` | TB6612FNG H-bridge, encoder counting |
| `lib/BalanceController/` | Cascaded PID |
| `lib/RecoveryManeuver/` | Open-loop wind-up and kick |
| `lib/RobotState/` | Shared state struct and mode enum |
| `include/config.h` | Pins, gains, limits. **Placeholders — tune before running** |

Architecture rationale is in [`../docs/architecture.md`](../docs/architecture.md); the
hand-tuning procedure is in [`../docs/tuning.md`](../docs/tuning.md).

## What stage 6 needs from this firmware

The identification stage cannot work with telemetry designed only for human debugging.
Before starting stage 6, this firmware needs:

1. **Timestamps from the device**, not from host arrival time. USB serial jitter is
   milliseconds and will be mistaken for actuator lag.
2. **Raw values alongside filtered ones.** Log the raw gyro and accelerometer, not just
   the fused angle — the fusion filter is itself a parameter you may want to identify.
3. **Commanded torque *and* measured current**, so the actuator model has both sides.
4. **An open-loop excitation mode** that plays a scripted signal — chirp, PRBS, multisine —
   with the balance controller disabled. Without this the robot is not identifiable; see
   stage 6.
5. **Deterministic loop timing**, with any missed deadline flagged in the log rather than
   silently absorbed.
6. **A logging rate at least 5× your fastest dynamics.** For a balancer, 200 Hz minimum,
   500 Hz preferred.

## Gotchas

- **Bench-test with wheels off the ground.** The recovery maneuver applies full torque.
- **Gains do not transfer between chassis.** The values in `config.h` are placeholders and
  a different battery position alone can invalidate them.
- **Watch for encoder aliasing at speed.** During the recovery kick the wheels spin far
  faster than during balancing, and a counting scheme that works at balance speeds may
  silently lose counts during the kick.
