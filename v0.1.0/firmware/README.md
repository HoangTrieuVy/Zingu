# v0.1.0 firmware — LQR state feedback

Arduino sketch for the ESP32. Open `BalanceBot_V4_LQR/BalanceBot_V4_LQR.ino` in the
Arduino IDE and flash it.

## First: secrets

The sketch joins your WiFi to serve its tuning UI. Credentials are not in the repo:

```sh
cd BalanceBot_V4_LQR
cp secrets.h.example secrets.h
# edit secrets.h with your own SSID and password
```

`secrets.h` is gitignored. Do not commit it.

## The controller

One flat state-feedback law — four gains, no inner/outer split:

```cpp
u = K1*angle + K2*angleRate + K3*wheelSpeed + K4*position;   // acceleration
velCmd += u * dt;                                            // steppers take velocity
```

| state | source | unit |
|---|---|---|
| `angle` | Kalman | ° |
| `angleRate` | gyro − estimated bias | °/s |
| `wheelSpeed` | commanded step rate | ksteps/s |
| `position` | steps counted in the ISR | ksteps |

All four gains are normally positive.

`u` is an **acceleration**. A stepper is a velocity device, so `u` is integrated into
`velCmd` before it reaches the wheels — holding a tilted robot up requires sustained
wheel acceleration, and a constant wheel speed produces no righting force at all.

## ARM vs ENGAGE

- **ARM** — you press it. The robot is now *allowed* to balance.
- **ENGAGE** — the robot decides: it engages within 2° of upright, and disengages past 40°.

So you can pick the robot up (motors stop) and stand it back down (it catches itself)
without touching your phone. `DISARM` stops it for good. The status line shows `[ON]`
only when actually engaged.

## Tuning — 4 gains, in this order

Start with **K3 = K4 = 0**. That gives pure angle stabilisation: it will balance but
drift. Get that solid first.

| step | gain | start | what you're looking for |
|---|---|---|---|
| 1 | **K1** angle | 1200 | Raise until it just oscillates, back off ~20% |
| 2 | **K2** rate | 60 | Raise until the oscillation damps out |
| 3 | **K3** speed | 1000 | Push it — it should stop instead of running away |
| 4 | **K4** position | 400 | It should now return to where it engaged |
| 5 | **targetAngle** | 0 | Trim out any residual steady drift |

Plot group 3 shows `K1ang K2rate K3vel K4pos u` — the four terms side by side, so you
can see which one is actually driving the wheels and which is doing nothing.

> The starting values are estimates derived from the integration scale, not
> measurements. Expect to move them by a factor of 2–3. The *order* matters far more
> than the starting numbers.

If a gain makes things worse as you raise it, that term's sign is wrong for your
wiring — negate it.

## What else is in there

20 kHz DDS stepper ISR, Kalman angle filter, step-counting odometry, a web tuning UI
on `balance2.local`, NVS gain persistence, plot groups and watchdogs. See
[`../docs/architecture.md`](../docs/architecture.md) and
[`../docs/tuning.md`](../docs/tuning.md).

## Provenance

This is the pre-split V2 LQR sketch reassembled into one file, recovered from the
Arduino build cache. It excludes later V3 work (rail/stall watchdog, boot reason,
no-cache headers, newer UI). An earlier cascade-PID controller exists but is not
carried in this repository.
