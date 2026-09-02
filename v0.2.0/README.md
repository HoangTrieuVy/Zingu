# Zingu v0.2.0 — Legs

**Status: in CAD.** Nothing here is fabricated or tested yet.

v0.1.0 is a rigid chassis bolted straight to two wheels — the shortest path to a
balancing robot. v0.2.0 puts a leg between the chassis and each wheel.

## What changes

- **Articulated legs.** A linkage on each side lets the body change height and ride
  over ground the rigid chassis simply scrapes across.
- **Larger spoked wheels.** More ground clearance and a longer contact arc to catch
  a fall with.
- **Redesigned chassis.** Electronics move into an enclosed box instead of riding
  exposed on the top plate.

The point is not the mechanism for its own sake: more degrees of freedom is exactly
where a hand-tuned PID stops being enough and a learned policy starts to earn its keep.

## Where things stand

| Directory | Status |
| --------- | ------ |
| `design/` | CAD assembly in progress |
| `print/` | Not started |
| `electronics/` | Carried over from v0.1.0, not yet revised |
| `firmware/` | Not started — will fork from `../v0.1.0/firmware` |
| `sim/` | Not started |
| `train/` | Not started |

Until a directory here has content, the v0.1.0 equivalent is the reference.
