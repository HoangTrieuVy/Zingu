# Zingu

A two-wheeled self-balancing robot that gets back up on its own — built end to end,
in the open, from the first CAD sketch to a policy running on real hardware.

Zingu is an inverted-pendulum robot built around an ESP32. It holds itself upright
with a cascaded PID controller, and when it falls over — because it always
eventually does — it detects the fall, waits for the chassis to settle, and
performs a wheel-torque kick-up to swing itself back into the balance envelope.

```
        IDLE ──arm──> RECOVERING ──in envelope──> BALANCING
                          ▲                            │
                          └────── fall detected ───────┘
                                       │
                                     FAULT ── (stall / overcurrent / IMU loss)
```

## What this repository is

Most robot repos give you the firmware and leave the other 90% as an exercise.
This one is organised as a **build path**: eight numbered stages that take you from
a blank CAD document to a trained policy deployed on hardware, with the simulation
model calibrated against the real machine rather than assumed.

The through-line is a claim worth testing: **a robot should ship as a simulatable,
identified asset, not just as a pile of parts and a binary.** Every stage below
either produces that asset or tightens it.

## The build path

| Stage | Directory | You end up with |
| ----- | --------- | --------------- |
| 1. Design | [`01-design/`](01-design/) | A parametric CAD assembly, a BOM, and parts that are actually printable |
| 2. Asset | [`02-asset/`](02-asset/) | CAD exported to MJCF / URDF / USD with real mass properties |
| 3. Sim | [`03-sim/`](03-sim/) | A *nominal* MuJoCo model that balances under a hand-written controller |
| 4. Fabricate | [`04-fabricate/`](04-fabricate/) | Printed, assembled, wired hardware |
| 5. Firmware | [`05-firmware/`](05-firmware/) | ESP32 firmware that balances, recovers, and streams telemetry |
| 6. Identify | [`06-identify/`](06-identify/) | Measured actuator model, inertial parameters, latency — and a posterior over them |
| 7. Train | [`07-train/`](07-train/) | A policy trained in the *identified* sim, randomised over that posterior |
| 8. Deploy | [`08-deploy/`](08-deploy/) | That policy running on the real robot, with the sim-to-real gap measured |

## It is a loop, not a line

The numbering is a reading order, not a one-way street. The interesting part of
this project is the cycle between stages 3, 6 and 7:

```
      ┌──────────────────────────────────────────────────┐
      │                                                  │
  01 Design ──> 02 Asset ──> 03 Sim ──> 07 Train ──> 08 Deploy
                                ▲                        │
                                │                        │
                                └──── 06 Identify <───────┘
                                            ▲
                                            │
                        04 Fabricate ──> 05 Firmware
```

A sim built from CAD alone is a *guess*: it has the geometry right and the
dynamics wrong. Stage 6 replaces the guesses with measurements, and stage 7
trains against a distribution of plausible robots rather than one idealised one.
Every trip around the loop should shrink the gap you measure in stage 8.

## Start here

New to the project? Read the stage READMEs in order — each one is written as a
chapter, with prerequisites, steps, and the specific ways that stage goes wrong.
You do not need hardware to get through stage 3.

If you only want the robot running and don't care about simulation, stages
1 → 4 → 5 are a complete conventional build.

## Cross-cutting docs

| Path | Contents |
| ---- | -------- |
| [`docs/architecture.md`](docs/architecture.md) | Control loop, angle estimation, state machine, module boundaries |
| [`docs/hardware.md`](docs/hardware.md) | Bill of materials, pinout, power, geometry |
| [`docs/tuning.md`](docs/tuning.md) | Hand-tuning procedure for the PID gains |
| [`docs/references.md`](docs/references.md) | Prior art — balancing robots, wheeled bipeds, sysid |
| [`tools/`](tools/) | Host-side scripts (live telemetry plotting) |
| `data/` | Calibration datasets and telemetry captures (bulk contents untracked) |

## Simulator choice

Stages 3, 6 and 7 use **MuJoCo**. It runs natively on macOS and Linux, its
actuator model is the right one for system identification, and MJX gives you
GPU-parallel rollouts when you need them.

NVIDIA **Newton** (1.0 GA, built on Warp + OpenUSD) with **Isaac Lab** is the
intended destination once this robot grows a manipulator or needs thousands of
parallel environments. That move is deliberately deferred, not ruled out — see
[`02-asset/README.md`](02-asset/) for how the asset pipeline keeps that door open.
Note that Isaac Sim requires Linux or Windows with an NVIDIA RTX GPU.

## Status

Early. The control structure, state machine and module boundaries exist in
firmware; gains in `05-firmware/include/config.h` are placeholders and **must** be
tuned against your own chassis before the robot moves under its own power. Stages
2, 3, 6, 7 and 8 are scaffolded roadmaps, not finished code.

## Safety

Zingu's recovery maneuver deliberately applies full motor torque. Bench-test with
the wheels off the ground, keep the battery accessible, and stay clear of the
kick-up arc. The bench rig in stage 6 spins a weighted disc at speed — clamp it
down and wear eye protection.

## License

MIT — see [LICENSE](LICENSE).
