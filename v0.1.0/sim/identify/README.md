# Stage 6 — Identify

**Goal:** replace the invented numbers in the sim with measured ones, and produce a
*posterior* over the parameters rather than a single point estimate.

**Prerequisites:** working hardware (stage 4), telemetry and open-loop excitation mode
(stage 5), nominal sim (stage 3).

This is the stage that makes the rest of the project mean anything. A sim built from CAD
has the geometry right and the dynamics wrong; everything trained on it inherits that.

## Identify hierarchically, never all at once

Joint identification of every parameter simultaneously is where these projects die. The
parameters are strongly correlated — mass trades against COM height, friction against
torque constant, latency against damping — and a joint fit slides into a valley and stops.

| Step | Rig | What you get |
| ---- | --- | ------------ |
| 6a | One motor, known-inertia disc, clamped to bench | Torque constant, gearbox ratio and efficiency, Coulomb + viscous friction, current limit, torque–speed curve |
| 6b | Same rig, step and chirp inputs | Actuator bandwidth and **total latency** — controller, comms, driver |
| 6c | Chassis pinned, wheels free | Wheel inertia, encoder scale, deadband |
| 6d | Free robot, open-loop excitation | Base inertial parameters, COM height, ground friction |

Design and protocol for each rig in `bench/`.

## The two traps

**A balance controller destroys identifiability.** A working controller regulates the
state to a fixed point, and at a fixed point there is almost no excitation — the data
looks clean and contains nothing. You must deliberately inject chirps, PRBS or multisine
through the open-loop mode, or run maneuvers that violate the controller's comfort zone.
Excitation design lives in `excitation/`.

**Individual link inertias are provably not identifiable.** Only certain linear
combinations — the *base parameters* — can be recovered from motion data. Trying to fit
all ten inertial parameters per body is not hard, it is impossible. Gautier & Khalil is
the canonical reference, and it also gives you condition-number-optimised exciting
trajectory design. Read it before collecting data, not after.

## Fitting procedure

Work in `fit/`.

1. **Screen first, optimise second.** Latin hypercube sample the parameter space, then run
   Morris screening or Sobol indices. Drop parameters the loss is insensitive to — they
   are unidentifiable, and keeping them only adds local minima. This *is* your parameter
   selection step, and it is principled rather than a guess.
2. **Choose a loss that survives divergence.** Naive L2 over long trajectories is useless
   for an unstable system: small parameter errors diverge and the loss becomes noise past
   a short horizon. Use either short-horizon prediction error with re-initialisation from
   real state, or **frequency-domain matching** — chirp in, compare spectra. For a
   balancer, frequency-domain is classical, cheap, and unusually effective.
3. **Weight residuals by measurement noise.** Maximum likelihood, not raw L2, or your IMU
   units will arbitrarily dominate your encoder units.
4. **Do not use coordinate descent.** Greedy per-dimension search fails on correlated
   parameters, which is all of them. Use CMA-ES — cheap, noise-tolerant, handles
   correlation — or gradients if you make the sim differentiable via MJX.
5. **Fit a posterior, not a point.** Point-calibrated sims are brittle in exactly the way
   that makes people give up on sim-to-real. Keep the spread and hand it to stage 7 as the
   domain-randomisation distribution. See BayesSim, SimOpt, DROPO.
6. **Hold out validation trajectories** the fit never saw. Report error on those, not on
   the fitting set.

## Outputs

| Path | Contents |
| ---- | -------- |
| `bench/` | Rig designs, wiring, and measurement protocols |
| `excitation/` | Excitation signal design and identifiability analysis |
| `fit/` | Loss definitions, sensitivity analysis, optimiser configs |
| `../data/calibration/` | Raw captured datasets |
| `../mjcf/` | Updated with identified parameters — the calibrated model |

## Gotchas

- **Make this re-runnable as one command.** The robot will change — new battery, new motor
  mount, worn tyres — and recalibration must be routine, not a project.
- **Latency is the parameter people forget and then lose weeks to.** On an inverted
  pendulum a 10 ms delay moves the stability margin more than a 10% inertia error.
- **Temperature drifts your motor constants.** Identify warm, at the duty cycle you
  actually run at, not from a cold start.
