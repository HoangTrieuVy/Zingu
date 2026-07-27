# References

Prior art and background material for Zingu. Cite these by their key (e.g. `[REMRC-V3]`)
from design notes, commit messages, and `docs/` prose.

Each entry records what it is actually useful *for* — the point is to avoid re-watching an
hour of video to find the one detail that mattered.

## Wheeled-leg / wheeled-biped balancing

These run the same inverted-pendulum control problem Zingu does, but add a variable-height
leg. Relevant to us mainly for how they handle the ground-contact and recovery cases.

- `[REMRC-V3]` — **Wheeled-leg self balancing robot ver. 3** — ReM-RC
  <https://www.youtube.com/watch?v=gHynbYIm1Rg>
  Third iteration of a wheeled-leg balancer. Useful as a reference point for what a
  refined version of this chassis class looks like.

- `[HATTORI-STRIDE]` — **STRIDE (Wheeled Biped V2) Balancing and Jumping** — Alex Hattori
  <https://www.youtube.com/watch?v=iQqKwUX6nv0>
  Wheeled biped that balances *and* jumps. The jump/landing recovery is the closest
  published analogue to Zingu's `RecoveryManeuver` kick-up: both need the controller to stay
  sane through a period where wheel contact is not guaranteed.

- `[SBS-WHEELLEG]` — **Try Making Wheeled-Legs Balancing Bot** — stepbystep-robotics
  <https://www.youtube.com/watch?v=iCJNfWUn47Q>
  Build-along treatment of the wheeled-leg configuration.

## Two-wheel inverted pendulum — theory and build

Directly comparable to Zingu's chassis.

- `[ROBONYX-THEORY]` — **How Self Balancing Robots Work! (Theory, Components, Design, PID)** — Robonyx
  <https://www.youtube.com/watch?v=J5Xd43LIFiU&t=109s>
  The theory reference of this set: PID structure, component selection, design rationale.
  Cite this when justifying the cascaded velocity-PI → pitch-PID arrangement in
  `BalanceController` or the gyro-rate D term.

- `[BSS-ARDUINO]` — **I built a self-balancing robot from scratch (Arduino Based)** — Build Some Stuff
  <https://www.youtube.com/watch?v=K1lzzVGCzAQ>
  End-to-end scratch build on Arduino. Useful for the practical failure modes — motor
  deadband, encoder noise, IMU mounting — that `docs/tuning.md` has to account for.

## Open-source platforms

- `[BDX-R]` — **BDX-R: An Open-Source RL Biped Platform** — <https://github.com/BDX-R>
  Docs: <https://bdx-r.github.io/>
  Community-driven open biped platform for practical robot learning and reproducible
  sim-to-real, publishing robot models and simulation assets. Relevant repos:
  - `BDX-R-CAD` — CAD for the robot
  - `BDX-R-IsaacLab` — Isaac Lab reinforcement-learning environment (Python)
  - `BDX-R-MjLab` — mjlab reinforcement-learning environment (Python)

  Different control philosophy from Zingu (learned policies vs. hand-tuned cascaded PID),
  so cite it as the contrast case — and as the place to look if Zingu ever grows a
  simulation or sim-to-real path.

---

**Note on scope:** every entry above is prior art consulted for background. None of it is
vendored into this repo, and no code here is derived from it. If that changes — if a
control law, gain set, or geometry is taken from one of these — say so at the point of use
and not only here.
