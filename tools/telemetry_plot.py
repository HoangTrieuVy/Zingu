#!/usr/bin/env python3
"""Live plot of Zingu telemetry.

The firmware emits one CSV line per telemetry tick on the serial port:

    T,<millis>,<state>,<pitch>,<setpoint>,<rate>,<velocity>,<accel_g>

Pipe the serial monitor into this script:

    pio device monitor | python3 tools/telemetry_plot.py

Or read the port directly:

    python3 tools/telemetry_plot.py --port /dev/ttyUSB0

Add --csv run.csv to save the parsed stream for later comparison.
"""

from __future__ import annotations

import argparse
import csv
import sys
from collections import deque
from dataclasses import dataclass
from typing import Iterator, TextIO

WINDOW_SECONDS = 10.0
TELEMETRY_PREFIX = "T"
FIELD_COUNT = 8

STATE_COLORS = {
    "IDLE": "#9e9e9e",
    "BALANCING": "#2e7d32",
    "RECOVERING": "#ef6c00",
    "FAULT": "#c62828",
}


@dataclass(frozen=True)
class Sample:
    t_ms: int
    state: str
    pitch_deg: float
    setpoint_deg: float
    rate_dps: float
    velocity_rps: float
    accel_g: float


def parse_line(line: str) -> Sample | None:
    """Return a Sample, or None for boot messages and partial lines."""
    parts = line.strip().split(",")
    if len(parts) != FIELD_COUNT or parts[0] != TELEMETRY_PREFIX:
        return None
    try:
        return Sample(
            t_ms=int(parts[1]),
            state=parts[2],
            pitch_deg=float(parts[3]),
            setpoint_deg=float(parts[4]),
            rate_dps=float(parts[5]),
            velocity_rps=float(parts[6]),
            accel_g=float(parts[7]),
        )
    except ValueError:
        # A line split by a reset mid-transmission. Dropping one sample is
        # cheaper than crashing a session that has been running for an hour.
        return None


def read_stream(stream: TextIO) -> Iterator[Sample]:
    for line in stream:
        sample = parse_line(line)
        if sample is not None:
            yield sample


def read_serial(port: str, baud: int) -> Iterator[Sample]:
    try:
        import serial  # type: ignore
    except ImportError:
        sys.exit("--port needs pyserial: pip install pyserial")

    with serial.Serial(port, baud, timeout=1) as handle:
        while True:
            raw = handle.readline().decode("utf-8", errors="replace")
            if not raw:
                continue
            sample = parse_line(raw)
            if sample is not None:
                yield sample


def run_plot(samples: Iterator[Sample], csv_path: str | None) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        sys.exit("plotting needs matplotlib: pip install matplotlib")

    from matplotlib.animation import FuncAnimation

    times: deque[float] = deque()
    pitch: deque[float] = deque()
    setpoint: deque[float] = deque()
    velocity: deque[float] = deque()

    csv_file = open(csv_path, "w", newline="") if csv_path else None
    writer = csv.writer(csv_file) if csv_file else None
    if writer:
        writer.writerow(
            ["t_ms", "state", "pitch", "setpoint", "rate", "velocity", "accel_g"]
        )

    fig, (ax_angle, ax_vel) = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
    fig.suptitle("Zingu telemetry")

    (pitch_line,) = ax_angle.plot([], [], label="pitch", lw=1.4)
    (setpoint_line,) = ax_angle.plot([], [], label="setpoint", lw=1.0, ls="--")
    ax_angle.set_ylabel("degrees")
    ax_angle.axhline(0.0, color="#bbbbbb", lw=0.8)
    ax_angle.legend(loc="upper right")
    ax_angle.grid(alpha=0.3)

    (velocity_line,) = ax_vel.plot([], [], label="wheel velocity", lw=1.2)
    ax_vel.set_ylabel("rev/s")
    ax_vel.set_xlabel("seconds")
    ax_vel.axhline(0.0, color="#bbbbbb", lw=0.8)
    ax_vel.grid(alpha=0.3)

    state_text = ax_angle.text(
        0.01, 0.92, "", transform=ax_angle.transAxes, fontweight="bold"
    )

    def update(_frame: int):
        # Drain whatever arrived since the last frame rather than blocking on a
        # fixed count, so the plot keeps up with a fast robot and does not stall
        # on a quiet one.
        for _ in range(64):
            try:
                sample = next(samples)
            except StopIteration:
                break

            if writer:
                writer.writerow(
                    [
                        sample.t_ms,
                        sample.state,
                        sample.pitch_deg,
                        sample.setpoint_deg,
                        sample.rate_dps,
                        sample.velocity_rps,
                        sample.accel_g,
                    ]
                )

            t = sample.t_ms / 1000.0
            times.append(t)
            pitch.append(sample.pitch_deg)
            setpoint.append(sample.setpoint_deg)
            velocity.append(sample.velocity_rps)

            state_text.set_text(sample.state)
            state_text.set_color(STATE_COLORS.get(sample.state, "#000000"))

            while times and (t - times[0]) > WINDOW_SECONDS:
                times.popleft()
                pitch.popleft()
                setpoint.popleft()
                velocity.popleft()

        if not times:
            return pitch_line, setpoint_line, velocity_line, state_text

        pitch_line.set_data(times, pitch)
        setpoint_line.set_data(times, setpoint)
        velocity_line.set_data(times, velocity)

        ax_angle.set_xlim(times[0], max(times[-1], times[0] + 1.0))
        ax_angle.relim()
        ax_angle.autoscale_view(scalex=False)
        ax_vel.relim()
        ax_vel.autoscale_view(scalex=False)

        return pitch_line, setpoint_line, velocity_line, state_text

    animation = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    try:
        plt.show()
    finally:
        # Keep a reference alive until show() returns; matplotlib drops the
        # animation otherwise and the plot silently freezes.
        del animation
        if csv_file:
            csv_file.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="serial port; omit to read stdin")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--csv", help="also write parsed samples to this file")
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="print samples instead of plotting (no matplotlib needed)",
    )
    args = parser.parse_args()

    samples = read_serial(args.port, args.baud) if args.port else read_stream(sys.stdin)

    if args.no_plot:
        for sample in samples:
            print(
                f"{sample.t_ms:>8} {sample.state:<10} "
                f"pitch={sample.pitch_deg:>7.2f} set={sample.setpoint_deg:>6.2f} "
                f"vel={sample.velocity_rps:>6.3f}"
            )
        return

    run_plot(samples, args.csv)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
