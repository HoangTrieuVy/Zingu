# Architecture

## Control loop

One fixed-rate loop at 200 Hz does everything. There is no RTOS task split: the
whole tick — I2C read, filter, control, motor write — comfortably fits inside
5 ms on an ESP32, and a single loop removes every question about what happens
when the sensor thread and the control thread disagree about time.

```
                 ┌──────────────────────────── 200 Hz ────────────────────────────┐
                 │                                                                │
  MPU-6050 ──> Imu::update ──> pitch, pitch rate ──┐                               │
                                                    ├──> state machine ──> motors  │
  Encoders ──> MotorDriver::update ──> velocity ───┘                               │
                 │                                                                │
                 └────────────────────────────────────────────────────────────────┘
```

The scheduler advances a deadline by a fixed period rather than sleeping for a
fixed duration, so a tick that runs long is absorbed instead of shifting every
tick after it.

## Angle estimation

A complementary filter, not a Kalman filter. The accelerometer gives an absolute
but noisy tilt; the gyro gives a clean rate that drifts. Weighting them at
α = 0.98 gives a ~2.5 s crossover: gyro dominates the fast dynamics the
controller cares about, accelerometer slowly pulls out the drift.

A Kalman filter would be marginally better and considerably harder to explain
and tune. If you want one, `Imu` is the only class that would change.

Two details that matter more than the filter choice:

- **Pitch comes from `atan2`, not `asin`.** The robot spends real time lying
  down during recovery, well outside ±90°, where `asin` folds back on itself.
- **The D term uses the raw gyro rate**, not a difference of the filtered angle.
  Differencing a filtered signal amplifies exactly the noise the filter left
  behind; the gyro measures the derivative directly.

## Cascaded control

```
target velocity ──> ( − ) ──> [ PI ] ──> pitch setpoint ──> ( − ) ──> [ PID ] ──> duty
                      ▲                                       ▲
              measured velocity                        measured pitch
```

The outer loop's output is a *lean angle*, not a motor command. This is the part
that surprises people: to stop moving forward, the robot leans further forward
first, drives its wheels back under the centre of mass, and then comes upright.
Commanding the motors directly from velocity error would fight the inner loop and
the robot would ride away from itself.

The outer loop is deliberately slow (small gains, ±6° authority). A fast outer
loop competes with the inner loop for the same actuator and the two oscillate
against each other.

Anti-windup is a hard clamp on both integrators. Not elegant, but it is the only
windup mechanism in the system, which makes its behaviour easy to reason about
when the robot does something strange.

## State machine

```
                      ┌─────────────────────────────┐
                      │                             │
   boot ──> IDLE ──stood upright──> BALANCING        │
              ▲                       │              │
              │                  tilt > 42°          │
              │                       ▼              │
              │                  RECOVERING ─caught──┘
              │                       │
              │              attempts exhausted
              │                       ▼
              └──── power cycle ─── FAULT <─── IMU lost
```

The asymmetry between the thresholds is intentional. The robot gives up
balancing at 42° but only *accepts* a recovery at 12° with the rate below
90°/s. If re-entry used the same 42°, a kick that merely got the robot into the
old envelope would be handed to a balance controller with no authority left, and
it would fall again immediately — a loop that looks like the robot flailing.

## Recovery maneuver

Wheels cannot push the body up. Their reaction torque can.

| Phase  | Wheels                       | What the chassis does           |
| ------ | ---------------------------- | ------------------------------- |
| SETTLE | coast                        | stops rocking                   |
| WINDUP | torque *away* from the kick  | rocks up onto the pivot edge    |
| KICK   | full torque *toward* upright | body whips through vertical     |
| COAST  | free                         | arrives on its own momentum     |

Phases are timed, not sensed. During a flip the accelerometer reads centripetal
and impact acceleration rather than gravity, so the angle estimate is briefly
worthless — closing the loop on it mid-maneuver would make things worse, not
better. The move is short enough that open-loop timing is reliable, and success
is checked once at the end.

Braking during COAST is a common mistake: it plants the wheels and stops the
swing short of vertical.

## Module boundaries

| Module             | Owns                                      | Knows nothing about        |
| ------------------ | ----------------------------------------- | -------------------------- |
| `Imu`              | I2C registers, filter                     | motors, states             |
| `MotorDriver`      | PWM channels, direction pins, encoder ISRs| angles, control            |
| `BalanceController`| the two PID loops                         | hardware, states           |
| `RecoveryManeuver` | kick-up phase timing                      | angle estimation, PID      |
| `main.cpp`         | the state machine and the schedule        | register maps, gains       |

Every tuning constant lives in `include/config.h`. No module reaches for a
magic number of its own.

## Deliberate omissions

- **No steering.** One duty value drives both wheels. Differential drive belongs
  above the balance loop and does not exist yet.
- **No battery monitoring.** Gains that work at 8.4 V are soggy at 7.0 V; a real
  build should scale duty by measured voltage.
- **No stall detection.** A wheel jammed against an obstacle currently looks like
  a robot that keeps failing recovery until the attempt budget runs out.
