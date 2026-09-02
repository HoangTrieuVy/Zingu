# Tuning

The gains shipped in `firmware/include/config.h` are placeholders. They will not
balance your robot. Work through this in order — tuning the outer loop before
the inner loop is the most common way to waste an afternoon.

**Throughout: keep the robot on a stand with the wheels free, until step 4.**

## 0. Signs first

Do the four assembly checks in [hardware.md](hardware.md#assembly-checks-before-first-power-on)
and set `kPitchOffsetDeg`. A sign error looks exactly like bad gains, except no
amount of tuning fixes it. Do not skip this.

## 1. Inner loop: Kp

Zero out everything else:

```c
constexpr float kPitchKi = 0.0f;
constexpr float kPitchKd = 0.0f;
constexpr float kVelocityKp = 0.0f;
constexpr float kVelocityKi = 0.0f;
```

Raise `kPitchKp` until the robot oscillates steadily around vertical when you
nudge it — a consistent buzz, not a growing divergence. Then back off to about
60% of that value.

- Too low: the robot leans and falls without ever really trying.
- Too high: fast, constant vibration; the chassis hums.

## 2. Inner loop: Kd

`kPitchKd` damps the oscillation Kp left behind. Raise it until the nudge
response settles in roughly one overshoot.

Because D is taken from the gyro rate directly, useful values here are small —
two to three orders of magnitude below Kp. Too much D makes the robot feel
sluggish and reintroduces a high-frequency jitter as it amplifies gyro noise.

## 3. Inner loop: Ki

`kPitchKi` removes the steady lean left by an imperfect `kPitchOffsetDeg` and by
weight distribution. Raise it until the robot stops drifting slowly in one
direction while holding station.

Too much Ki produces a slow rocking oscillation, period of a second or more —
distinctly slower than the Kp buzz, which is how you tell them apart.

## 4. Floor test

Now put it on the ground. The robot should hold position but will probably drift
around — the outer loop is still disabled and nothing is regulating velocity yet.

Expect falls. Recovery is not tuned either; catch it by hand for now.

## 5. Outer loop

Bring up `kVelocityKp` first, then `kVelocityKi`. Both are small numbers. The
goal is a robot that returns to where it started after a shove, without
overshooting past it.

Symptoms:

- **Drifts away steadily** — velocity gains too low.
- **Slow surge back and forth**, period of a second or two — velocity gains too
  high. The outer loop is competing with the inner loop. Lower it, and do not
  compensate by raising the inner loop.
- **Leans hard and takes off** — sign error in the outer loop. The negation in
  `BalanceController::update` is deliberate; verify it against your pitch sign
  convention.

`kVelocityPitchLimitDeg` caps how far the outer loop may lean the robot. Raise it
only once everything else is stable; it is a safety limit, not a tuning knob.

## 6. Recovery

Different problem, different method. Recovery is open-loop, so you tune the four
timing constants by watching, not by reading numbers.

Start from a fallen robot on the floor and iterate:

| What you see                                    | What to change                          |
| ----------------------------------------------- | --------------------------------------- |
| Barely lifts; wheels spin uselessly             | ↑ `kRecoveryKickDuty`, ↑ `kRecoveryKickMs` |
| Whips past vertical and falls the other way     | ↓ `kRecoveryKickMs`                     |
| Lifts but stalls just short of upright          | ↑ `kRecoveryWindupMs`                   |
| Slides along the floor instead of rotating      | ↑ `kRecoveryWindupDuty` — needs more grip against the ground first |
| Arrives upright but the balance loop drops it   | ↑ `kRecoveryCoastMs`, or loosen `kUprightRateDegPerSec` |

Tune wind-up before kick. The wind-up phase is what plants the chassis on its
pivot edge; without it the kick just spins the wheels.

If it lands upright but immediately falls again, the problem is usually that it
arrives with too much angular rate, not that it arrives at the wrong angle.
Lengthening COAST lets the rate bleed off before the check runs.

## 7. Watch the telemetry

```bash
pio device monitor | python3 ../tools/telemetry_plot.py
```

The plot shows measured pitch against the setpoint the outer loop is asking for.
The gap between those two lines is what the inner loop is being asked to close —
if the setpoint is thrashing, the problem is upstream of the inner loop no matter
how the robot looks.

## Reference: what each gain does

| Constant                   | Effect when raised                              |
| -------------------------- | ----------------------------------------------- |
| `kPitchKp`                 | Stiffer response, eventually high-frequency buzz|
| `kPitchKi`                 | Kills steady lean, eventually slow rocking      |
| `kPitchKd`                 | Damps overshoot, eventually jitter and sluggishness |
| `kVelocityKp`              | Holds station harder, eventually slow surging   |
| `kVelocityKi`              | Removes long-term drift, eventually surging     |
| `kComplementaryAlpha`      | Trusts the gyro more; smoother but drifts       |
| `kMotorDeadbandDuty`       | Removes the flat spot; too high and it twitches |
