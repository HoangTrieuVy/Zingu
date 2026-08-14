# Stage 3 — Sim

**Goal:** a *nominal* MuJoCo model of Zingu that balances under a hand-written controller.
Nominal means "built from CAD priors and honest guesses" — it is not yet calibrated, and
you should not trust it for anything except structural sanity.

**Prerequisites:** stage 2 MJCF. No hardware needed — this stage is entirely on your laptop.

## Steps

1. **Load and look at it.** `python -m mujoco.viewer --mjcf=...`. Check the robot is the
   right size, the wheels spin about the right axes, and nothing is intersecting at rest.
2. **Drop test.** Disable actuators, drop the robot from 10 cm, watch it settle. It should
   fall over and come to rest without jitter, tunnelling or explosion. Jitter here means
   bad collision geometry or a timestep that's too large.
3. **Free-spin test.** Apply constant torque to one wheel with the chassis pinned. Angular
   acceleration should match `τ / I_wheel` from CAD. If it doesn't, your inertia export
   is wrong — go back to stage 2 rather than compensating here.
4. **Port the cascaded PID** from [`../05-firmware/`](../05-firmware/) — same structure,
   same gain semantics, so tuning insight transfers in both directions.
5. **Balance it.** Tune until it holds upright and rejects a shove. Record the gains; they
   are the baseline that stage 7's learned policy has to beat.
6. **Build scenes** in `scenes/` — flat floor, ramp, rough terrain, shove-disturbance. These
   become the training and evaluation environments later.
7. **Write validation scripts** in `validation/` that assert the above as tests, so a
   regenerated asset can't silently break the model.

## Outputs

| Path | Contents |
| ---- | -------- |
| `mjcf/` | Working model — scene wrappers, sensors, actuator declarations |
| `scenes/` | Terrain and disturbance variants for training and eval |
| `validation/` | Automated sanity checks against the model |

## Gotchas

- **The nominal model will not transfer, and that is expected.** Its friction, actuator
  torque curve, and latency are all invented. Stage 6 fixes that. Do not burn weeks
  hand-tuning a model you're about to replace with measurements.
- **Timestep and solver matter more than you'd like.** Start at 1 ms with the implicit
  integrator. If contact looks mushy, that's `solref`/`solimp`, not your controller.
- **Add the sensors you actually have.** Model the MPU-6050 as a gyro plus accelerometer
  at their real mounting pose, and the encoders as quantised joint position — not perfect
  state. A policy trained on perfect state is a policy that cannot be deployed.
- **Model latency from the start.** Even a placeholder one-step delay changes the
  stability margin of an inverted pendulum substantially. A sim with zero delay teaches
  the controller habits reality will punish.
