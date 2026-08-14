# Stage 8 — Deploy

**Goal:** run the stage 7 policy on real hardware, and **measure** the sim-to-real gap
instead of describing it.

**Prerequisites:** trained policy (stage 7), working robot (stages 4–5).

## Steps

1. **Decide where the policy runs.** On the ESP32 if it fits — quantised MLP, no dynamic
   allocation, bounded inference time. Otherwise on a tethered host with the ESP32 as an
   I/O bridge, and then *measure the added latency* and feed it back to stage 6, because
   you have just changed the plant.
2. **Verify inference determinism.** The same observation must produce the same action on
   device as in Python. Float precision and quantisation differences are a real and
   deeply confusing source of divergence.
3. **Bench first, wheels off the ground.** Confirm the policy produces sane torque commands
   before it can move anything.
4. **Keep the PID as a supervisor.** Run the learned policy inside the existing state
   machine with the classical controller as fallback on fault, tilt-limit, or timeout.
   Never let an untested policy be the only thing between the robot and the floor.
5. **Replay identical excitation** on robot and in sim, and overlay the trajectories. This
   is the actual sim-to-real gap measurement, and it is a number, not an impression.
6. **Feed the residual back to stage 6.** Where sim and reality diverge tells you which
   parameter is still wrong. That is one full trip around the loop.
7. **Log everything from real runs** into `../data/telemetry/`. Real-world trajectories are
   the scarcest asset in this whole project.

## Outputs

| Path | Contents |
| ---- | -------- |
| `eval/` | Deployment scripts, overlay/comparison tooling, gap metrics |
| `../data/telemetry/` | Real-robot logs from deployment runs |

## The metric that matters

Pick one number and track it across loop iterations. Suggested: **RMS state divergence
between sim and real over a fixed 5-second excitation**, from identical initial conditions.

Every trip around Design → Sim → Identify → Train → Deploy should shrink it. If it isn't
shrinking, the loop is not closing and something upstream is wrong — that is a finding,
not a failure.

## Gotchas

- **Battery voltage sag changes the plant mid-run.** A policy trained at nominal voltage
  degrades as the pack drains. Either include voltage in the observation or randomise
  torque scale over the real discharge range in stage 7.
- **The first real fall will break something.** Print spares of the fragile parts before
  the first untethered run, not after.
- **Do not tune the real robot to match the sim.** The direction of correction is always
  sim toward reality. Adjusting hardware to flatter the model destroys the only ground
  truth you have.
