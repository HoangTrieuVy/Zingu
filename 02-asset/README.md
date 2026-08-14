# Stage 2 — Asset

**Goal:** turn the CAD assembly into a simulator-readable robot description, generated
by a script, never hand-edited.

**Prerequisites:** stage 1 assembly with mates and materials assigned.

## The one rule

**CAD is the single source of truth. Everything else is generated.** The moment you
hand-edit a URDF to fix something, you have two robots that disagree, and you will spend
a week finding out which one lied to you.

## Format strategy

| Format | Role here | Why |
| ------ | --------- | --- |
| **MJCF** | Dynamics-authoritative | MuJoCo's actuator, tendon and contact model is the one worth identifying against |
| **URDF** | Lossy downstream export | Universal interop; no closed loops, no real actuator model |
| **USD** | Scene composition | The path to Isaac Lab / Newton when the time comes |

Generate MJCF first, derive the rest. Isaac Sim's URDF and MJCF importers are built on
the USD Exchange SDK and structure imported assets to run under either PhysX or Newton,
so a clean MJCF is a valid on-ramp to the NVIDIA stack later.

## Steps

1. **Extract mass properties per body** from CAD — mass, COM, full inertia tensor about
   the COM, in a documented frame.
2. **Extract the kinematic tree** from the mates. For Zingu this is trivial: base plus two
   revolute wheel joints. Record joint axes and limits.
3. **Generate collision geometry separately from visual geometry.** Visual can be the
   tessellated mesh; collision should be primitives (cylinders for wheels, boxes for the
   deck) wherever possible. Convex decomposition (CoACD, V-HACD) only where a primitive
   genuinely won't do.
4. **Emit MJCF** into `models/`, with actuators declared but *unparameterised* — stage 6
   fills in the numbers.
5. **Write the export as a script** in `export/`, committed, re-runnable. The chassis will
   change; regeneration must be one command.
6. **Diff-check regeneration.** Re-running the export on unchanged CAD must produce a
   byte-identical file, or you can't tell real changes from noise.

## Outputs

| Path | Contents |
| ---- | -------- |
| `export/` | Export and conversion scripts (CAD → MJCF → URDF/USD) |
| `models/` | Generated robot descriptions. Generated — do not hand-edit |

## Gotchas

- **Wheel collision geometry decides everything.** A balancer lives or dies on wheel-ground
  contact. Use an exact cylinder primitive, not a 64-facet mesh that produces a bumpy ride
  and phantom torque ripple.
- **Tiny CAD features destroy the solver.** Fillets, chamfers, screw threads and M3 holes
  tessellate into thousands of near-degenerate triangles. Suppress them before export.
- **Check the inertia tensor is physically valid.** Principal moments must satisfy the
  triangle inequality. Bad exports silently produce impossible inertias, and MuJoCo will
  either refuse to load or behave insanely.

## Useful tooling

`onshape-to-robot` (Onshape → URDF/MJCF directly from mates), MuJoCo's built-in URDF
loader, `urdf2mjcf`, Isaac Sim's importers.
