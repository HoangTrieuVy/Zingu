# Stage 7 — Train

**Goal:** a policy trained in the *identified* sim, randomised over the posterior from
stage 6, that beats the hand-tuned PID baseline on robustness rather than on nominal
performance.

**Prerequisites:** calibrated model (stage 6), scenes (stage 3).

## Steps

1. **Define the task properly.** Not "stay upright" — a PID already does that, and better.
   Aim where learning actually wins: recovery from arbitrary fall poses, rough terrain,
   large disturbances, payload changes, low-battery torque droop.
2. **Match the observation space to real sensors exactly.** Quantised encoder counts, noisy
   gyro at the real mounting pose, real update rate, real latency. Every mismatch here is
   a sim-to-real gap you are building in deliberately.
3. **Randomise over the stage 6 posterior**, not over invented ±20% ranges. This is the
   payoff for having done identification properly: the randomisation distribution is
   *measured*, so it covers reality without being needlessly wide. Configs in `configs/`.
4. **Build the env** in `envs/` — MJX for GPU-parallel rollouts, wrapping the stage 3
   scenes. Keep the reward and termination logic separate from the physics wrapper.
5. **Train.** PPO or SAC are both fine; this is a low-DOF problem and the algorithm is not
   the interesting part.
6. **Evaluate against the PID baseline** on held-out scenes. If it doesn't beat the
   baseline on disturbance rejection, the interesting result is *why* — usually an
   observation-space mismatch or a reward that rewards the wrong thing.
7. **Check policy smoothness before deploying.** A policy that chatters at the action
   limit will destroy real gearboxes even if it scores well in sim. Penalise action rate.

## Outputs

| Path | Contents |
| ---- | -------- |
| `envs/` | MJX environment, reward, termination, observation wrapper |
| `configs/` | Training hyperparameters and domain-randomisation ranges |

## Gotchas

- **Do not train on the nominal model.** It is a guess. Stage 6 exists for this reason.
- **Reward shaping will quietly teach the wrong behaviour.** A robot rewarded for staying
  upright learns to sit still; a robot rewarded for tracking velocity learns to lurch.
  Watch rollouts, don't just read curves.
- **Sim-only success proves nothing.** The number that matters is measured in stage 8.

## When to move to Isaac Lab + Newton

Newton 1.0 (Warp + OpenUSD, MuJoCo Warp as a core solver) with Isaac Lab is the right
destination when this project needs massively parallel environments or contact-rich
manipulation — realistically, when Zingu Brain grows an arm. Two things to know:

- Isaac Sim requires **Linux or Windows with an NVIDIA RTX GPU**. It does not run on macOS.
- Because MuJoCo Warp is a Newton solver, MJCF work and identified parameters carry
  forward. Starting in MuJoCo is not throwaway effort.

Porting *after* identification means arriving with a validated model instead of a guess.
