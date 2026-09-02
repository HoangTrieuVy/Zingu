# Stage 4 — Fabricate

**Goal:** printed, assembled, wired hardware that matches the CAD closely enough that the
model means something.

**Prerequisites:** stage 1 CAD, the BOM ordered and arrived.

## Steps

1. **Print the structural parts.** Deck, motor mounts, IMU mount, battery cradle. Profiles
   and per-part orientation notes in `print-profiles/`.
2. **Test-fit before committing.** Print motor mounts and axle bosses first and check them
   against the real motors. Tolerance errors found now cost an hour; found after a full
   print, a day.
3. **Assemble following `assembly/`.** Photograph each step as you go — this is the
   documentation you cannot reconstruct afterwards.
4. **Weigh everything.** Each printed part, each vendor part, then the fully assembled
   robot. Record it in `../design/bom/`.
5. **Reconcile mass against CAD.** Compute assembled-mass minus CAD-mass. That delta is
   unmodelled mass — wiring, glue, fasteners, solder. Distribute it in the model or, better,
   feed it in as a prior for stage 6.
6. **Measure what CAD can't tell you.** Actual wheel diameter under load (compressed
   rubber is smaller than nominal), actual wheelbase, actual COM height by balancing the
   powered-down chassis on an edge.
7. **Run the pre-power-on checks** in [`../docs/hardware.md`](../docs/hardware.md) before
   connecting the battery.

## Outputs

| Path | Contents |
| ---- | -------- |
| `print-profiles/` | Slicer settings, material and orientation notes per part |
| `assembly/` | Step-by-step build guide with photographs |

Plus, into stage 6: measured masses, measured geometry, and a documented CAD-vs-reality delta.

## Gotchas

- **Print orientation determines whether motor mounts survive.** Layer lines perpendicular
  to the load will delaminate the first time the robot falls, and the robot falls a lot.
- **Rubber wheels are smaller than the box says.** The vendor set in
  `../design/cad/vendor/wheels/` is labelled 65 mm and measures 68 mm unloaded, less
  under load. Wheel radius enters the balance dynamics directly — measure, don't trust.
- **Strain-relieve everything before the first fall.** Motor leads snapping off pads is
  the single most common failure on a robot designed to tip over on purpose.
- **Mount the IMU rigidly and record its pose.** A shock-mounted or misaligned IMU makes
  angle estimation lie, and a balancer's whole existence depends on that estimate.
