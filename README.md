# Zingu

A two-wheeled self-balancing robot that gets back up on its own — built end to end,
in the open, from the first CAD sketch to a policy running on real hardware.

Zingu is an inverted-pendulum robot built around an ESP32. It holds itself upright
with a cascaded PID controller, and when it falls over — because it always
eventually does — it detects the fall, waits for the chassis to settle, and
performs a wheel-torque kick-up to swing itself back into the balance envelope.

## What this repository is

Every version of Zingu is open source all the way down:

- **Sketches and CAD** — the original design files, not just an exported mesh
- **Printable parts** — STLs and print settings for the whole chassis
- **Electronics** — a full parts list, wiring and pinout
- **Firmware** — the ESP32 code that balances, recovers and streams telemetry
- **Simulation and RL** — the robot model and the training code behind any learned policy

The project grows **one robot version at a time**. Each version directory is a
complete robot with its own key feature — and, for the parts worth learning in
simulation before touching hardware, its own sim-to-real story.

## Versions

| Version | Feature | Status |
| ------- | ------- | ------ |
| [`v0.1.0/`](v0.1.0/) | Balance & self-recovery | Running on hardware |
| [`v0.2.0/`](v0.2.0/) | Legs | In CAD |
| _next_ | Perception — Zingu Brain | Planned |

Start with [`v0.1.0/`](v0.1.0/). Each version README says what is built, what is not,
and which directories you need if you only want the robot running.

## Layout

Every version directory has the same shape:

```
v0.1.0/
├── design/        sketches, CAD, bill of materials
├── print/         print profiles and assembly notes
├── electronics/   parts list, wiring, pinout
├── firmware/      ESP32 firmware (PlatformIO)
├── sim/           robot model and system identification
├── train/         RL configs, environments, policy evaluation
├── docs/          control architecture, tuning procedure
└── data/          calibration and telemetry captures
```

Shared across versions:

| Path | Contents |
| ---- | -------- |
| [`docs/references.md`](docs/references.md) | Prior art — balancing robots, wheeled bipeds, sysid |
| [`tools/`](tools/) | Host-side scripts (live telemetry plotting) |

## Simulator choice

Simulation uses **MuJoCo**. It runs natively on macOS and Linux, its actuator model is
the right one for system identification, and MJX gives you GPU-parallel rollouts when
you need them.

NVIDIA **Newton** with **Isaac Lab** is the intended destination once this robot grows
a manipulator or needs thousands of parallel environments. That move is deliberately
deferred, not ruled out. Note that Isaac Sim requires Linux or Windows with an NVIDIA
RTX GPU.

## Safety

Zingu's recovery maneuver deliberately applies full motor torque. Bench-test with
the wheels off the ground, keep the battery accessible, and stay clear of the
kick-up arc.

## License

MIT — see [LICENSE](LICENSE).
