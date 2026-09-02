# Stage 1 — Design

**Goal:** a CAD assembly that is simultaneously a manufacturing source, a mass-properties
source, and a kinematic description. Not three separate models that drift apart.

**Prerequisites:** a parametric CAD tool. Onshape, Fusion, SolidWorks and FreeCAD all
work; Onshape is the easiest to script an export from.

## Steps

1. **Fix the parameters before the geometry.** Wheel diameter, wheelbase, deck height,
   battery position, motor mount spacing. Drive everything downstream off these. A
   balancer's behaviour is dominated by COM height and wheel radius — you *will* want to
   sweep them, and you can only sweep what is parametric.
2. **Model the vendor parts you didn't design.** Motors, IMU board, H-bridge, battery.
   They don't need internal detail, but they need correct bounding volume and correct
   mass. See `cad/vendor/`.
3. **Build the assembly with real mates**, not floating positioned bodies. The mates are
   what become joints in stage 2. A wheel that is "placed" rather than "revolute-mated"
   exports as a welded lump.
4. **Assign materials per part.** PLA vs PETG vs aluminium vs the actual density of a
   LiPo cell. This is where your inertia tensor comes from, and it is the cheapest
   accuracy you will ever buy.
5. **Design for the printer** — see [`../print/`](../print/) for the
   constraints to design against, and iterate here rather than in the slicer.
6. **Write the BOM** as you go, in `bom/`, with supplier links and measured masses once
   parts arrive.

## Outputs

| Path | Contents |
| ---- | -------- |
| `cad/chassis/` | Parts you designed — source format plus STEP |
| `cad/vendor/` | Bought parts: motors, wheels, electronics |
| `bom/` | Bill of materials with masses, costs, supplier links |

## Gotchas

- **CAD mass is a good prior and a bad truth.** It will not include wiring harness, glue,
  solder, connectors, heat-shrink, or the real internal mass distribution of the battery.
  Expect a 5–15% underestimate. Weigh the assembled robot in stage 4 and record the delta.
- **Export STEP alongside the native format.** SolidWorks and Onshape files are hostage
  to their tools; STEP is what stage 2 and future-you can actually read.
- **Keep the origin meaningful.** Put the assembly origin at the wheel axle midpoint with
  Z up. Every downstream frame convention gets easier, and URDF/MJCF root placement
  becomes trivial rather than a source of sign errors.

## Note on the vendor wheel files

`cad/vendor/wheels/` holds SolidWorks parts, a STEP assembly, 3MF, and reference photos
for the 65 mm wheel set (~33 MB). It is currently **untracked**. Binary CAD in plain git
bloats the repository permanently — decide deliberately between Git LFS, a release
attachment, or leaving it local and documenting the source in the BOM.
