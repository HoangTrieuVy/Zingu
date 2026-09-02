# Hardware

## Bill of materials

| Qty | Part                                  | Notes                                        |
| --- | ------------------------------------- | -------------------------------------------- |
| 1   | ESP32 DevKitC (or any ESP32 dev board)| 240 MHz, hardware PWM, plenty of interrupts  |
| 1   | MPU-6050 breakout                     | 6-axis IMU on I2C                            |
| 1   | TB6612FNG dual H-bridge               | 1.2 A/channel continuous, 3.2 A peak         |
| 2   | N20 gearmotor, 6 V, ~200 RPM, encoder | quadrature encoder on the rear shaft         |
| 1   | 2S LiPo, 7.4 V, ≥1000 mAh, ≥20C       | the kick pulls several amps for ~200 ms      |
| 1   | 5 V buck converter, ≥1 A              | logic rail, separate from the motor rail     |
| 2   | Wheels, 65 mm                         | see the geometry notes below                 |
| 1   | Chassis, ~150 mm tall                 | rigid; flex reads as phantom oscillation     |

## Pinout

Defined in `firmware/include/config.h`. Change it there, not in the code.

| Function              | ESP32 pin |
| --------------------- | --------- |
| I2C SDA               | 21        |
| I2C SCL               | 22        |
| Motor L — PWM         | 25        |
| Motor L — IN1 / IN2   | 26 / 27   |
| Motor R — PWM         | 32        |
| Motor R — IN1 / IN2   | 33 / 14   |
| H-bridge STBY         | 13        |
| Encoder L — A / B     | 34 / 35   |
| Encoder R — A / B     | 36 / 39   |

GPIO 34–39 are **input-only** on the ESP32, which is exactly what encoders need,
and using them frees the output-capable pins for the motor bridge. Note they have
no internal pull-ups — if your encoder breakout does not include them, add 10 kΩ
pull-ups to 3.3 V or the counts will be noise.

## Power

Two rails, one ground.

```
  2S LiPo ─┬─> TB6612FNG VM  (motor rail, 7.4 V, noisy)
           └─> buck 5 V ──> ESP32 VIN ──> 3.3 V ──> MPU-6050, TB6612 VCC
```

Run the motors off the pack directly and the logic off the buck. Sharing one
rail means the current spike at the start of a kick browns out the ESP32
mid-maneuver, which presents as random reboots that only ever happen during
recovery. Put a 470 µF electrolytic across VM at the bridge.

Tie all grounds at a single point near the battery.

## Geometry

Two numbers dominate how hard the robot is to tune:

**Centre-of-mass height.** A taller robot falls more slowly — the natural period
goes as √(h/g) — which gives the controller more time to react. This is why a
short, stubby balancer is *harder*, not easier. Put the battery high if you can
do it without making the chassis top-heavy enough to be unrecoverable.

**Wheel diameter.** Bigger wheels move the contact patch further per motor
revolution, so they buy correction authority, but they also raise the axle and
lengthen the moment arm the recovery kick has to swing. 65 mm is a reasonable
compromise on a 150 mm chassis.

Mount the IMU as close to the wheel axle as possible and rigidly. Off-axis
mounting means the accelerometer sees tangential acceleration from the chassis
rotating, not just gravity, and the filter reads that as tilt that is not there.

## Assembly checks before first power-on

1. **Motor polarity.** In `IDLE`, briefly command a positive duty by hand. Both
   wheels must drive the robot *forward*. If one is reversed, swap its two motor
   leads rather than negating in code.
2. **Encoder direction.** Push the robot forward by hand and watch
   `wheelVelocityRps` in the telemetry stream. It must go positive. If it does
   not, swap that encoder's A and B lines.
3. **Pitch sign.** Tip the robot nose-down. `pitchDeg` must go positive. If not,
   the IMU is mounted rotated — fix the mounting, or negate in `Imu::update`.
4. **Pitch offset.** Hold the robot at its true balance point (find it by
   balancing it on the wheels with power off) and read `pitchDeg`. Put that
   number in `cfg::kPitchOffsetDeg`.

Getting any of these signs wrong makes the robot accelerate *into* the fall.
Check all four with the wheels off the ground.
