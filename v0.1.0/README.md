# Zingu v0.1.0 — Balance & self-recovery

**Status: running on hardware.** Gains in `firmware/include/config.h` are placeholders
and must be tuned against your own chassis before the robot moves under its own power.

A rigid chassis bolted straight to two wheels. It holds itself upright with a cascaded
PID controller, and when it falls it detects the fall, waits for the chassis to settle,
and kicks itself back up with wheel torque.

```
        IDLE ──arm──> RECOVERING ──in envelope──> BALANCING
                          ▲                            │
                          └────── fall detected ───────┘
                                       │
                                     FAULT ── (stall / overcurrent / IMU loss)
```

## What is here

| Directory | Contents |
| --------- | -------- |
| `design/` | Parametric CAD, vendor parts, bill of materials |
| `print/` | Print profiles and assembly notes |
| `electronics/` | Parts list, wiring, pinout, power |
| `firmware/` | ESP32 firmware — balance, recovery, telemetry (PlatformIO) |
| `sim/` | MuJoCo model, exported assets, system identification |
| `train/` | RL configs and environments, and policy evaluation |
| `docs/` | Control architecture and the hand-tuning procedure |
| `data/` | Calibration datasets and telemetry captures (bulk untracked) |

## Build it

If you only want the robot running, you need `design/` → `print/` → `electronics/` →
`firmware/`. `sim/` and `train/` are the simulation and learning side and need no hardware.

## Safety

The recovery maneuver deliberately applies full motor torque. Bench-test with the wheels
off the ground, keep the battery accessible, and stay clear of the kick-up arc.
