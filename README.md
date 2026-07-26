# Zingu

A two-wheeled self-balancing robot that gets back up on its own.

Zingu is an inverted-pendulum robot built around an ESP32. It holds itself upright
with a cascaded PID controller, and when it does fall over — because it always
eventually does — it detects the fall, waits for the chassis to settle, and
performs a wheel-torque kick-up to swing itself back into the balance envelope.

```
        IDLE ──arm──> RECOVERING ──in envelope──> BALANCING
                          ▲                            │
                          └────── fall detected ───────┘
                                       │
                                     FAULT ── (stall / overcurrent / IMU loss)
```

## Why "self-recovery" matters

Most hobby balancers are one-shot: knock them over and they lie there with the
wheels spinning until you pick them up. Self-recovery closes that loop. The robot
treats "fallen" as just another state, not an end condition, which means it can
run unattended and survive rough terrain, shoves, and its own tuning mistakes.

The recovery maneuver is a two-phase move:

1. **Wind-up** — spin the wheels hard *away* from the direction the robot needs
   to travel, driving the chassis up onto its back edge.
2. **Kick** — reverse torque at peak wind-up. The reaction torque rotates the
   body through the upright position, where the balance controller catches it.

Phase timings are open-loop and hardware-specific; see
[docs/tuning.md](docs/tuning.md).

## Repository layout

| Path                    | What lives there                                       |
| ----------------------- | ------------------------------------------------------ |
| `firmware/`             | PlatformIO project — the code that runs on the ESP32   |
| `firmware/lib/`         | Self-contained modules (IMU, motors, control, recovery)|
| `docs/`                 | Architecture, hardware BOM and wiring, tuning guide    |
| `tools/`                | Host-side scripts (live telemetry plotting)            |

## Quick start

```bash
# Install PlatformIO Core if you don't have it
pip install platformio

cd firmware
pio run                 # build
pio run -t upload       # flash the ESP32
pio device monitor      # watch telemetry at 115200 baud
```

On first boot the robot stays in `IDLE` and streams IMU readings. Lay it flat and
leave it still for five seconds — it calibrates the gyro bias, then arms itself.

## Hardware

An ESP32 dev board, an MPU-6050 IMU, a TB6612FNG dual H-bridge, two N20 gearmotors
with quadrature encoders, and a 2S LiPo. Full bill of materials, pinout and
wiring notes: [docs/hardware.md](docs/hardware.md).

## Status

Early scaffold. The control structure, state machine and module boundaries are in
place; gains in `include/config.h` are placeholders and **must** be tuned against
your own chassis before the robot is allowed to move under its own power.

## Safety

Zingu's recovery maneuver deliberately applies full motor torque. Bench-test with
the wheels off the ground, keep the battery accessible, and stay clear of the
kick-up arc.

## License

MIT — see [LICENSE](LICENSE).
