#!/usr/bin/env python3
"""Read MPU6050 JSON packets from serial and report basic sanity checks."""

import argparse
import json
import math
import statistics
import time
from typing import Dict, List

import serial
from serial.tools import list_ports


FIELDS = ("accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z")
GRAVITY_M_S2 = 9.80665


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Check whether the firmware is streaming sane MPU6050 accel/gyro JSON."
        )
    )
    parser.add_argument(
        "--port",
        default="/dev/ttyACM0",
        help="Serial port to read, for example /dev/ttyACM0 or /dev/ttyUSB0.",
    )
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate.")
    parser.add_argument(
        "--seconds",
        type=float,
        default=8.0,
        help="How long to collect samples.",
    )
    parser.add_argument(
        "--min-rate",
        type=float,
        default=7.0,
        help="Minimum acceptable sample rate in Hz.",
    )
    parser.add_argument(
        "--max-rate",
        type=float,
        default=13.0,
        help="Maximum expected sample rate in Hz.",
    )
    parser.add_argument(
        "--gravity-tolerance",
        type=float,
        default=2.0,
        help="Allowed stationary accel norm error in m/s^2.",
    )
    parser.add_argument(
        "--stationary-gyro-limit",
        type=float,
        default=0.5,
        help="Expected max gyro norm in rad/s while the MPU is still.",
    )
    return parser.parse_args()


def print_available_ports() -> None:
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found by pyserial.")
        return

    print("Available serial ports:")
    for port in ports:
        print(f"  {port.device}: {port.description}")


def payload_from_line(line: bytes) -> Dict[str, float]:
    packet = line.decode("utf-8", errors="replace").strip()
    payload = json.loads(packet)
    missing = [field for field in FIELDS if field not in payload]
    if missing:
        raise ValueError(f"missing fields: {', '.join(missing)}")

    return {field: float(payload[field]) for field in FIELDS}


def collect_samples(args: argparse.Namespace) -> List[Dict[str, float]]:
    samples: List[Dict[str, float]] = []
    malformed_count = 0
    started_at = time.monotonic()

    with serial.Serial(args.port, args.baud, timeout=0.5) as ser:
        ser.reset_input_buffer()
        while time.monotonic() - started_at < args.seconds:
            line = ser.readline()
            if not line:
                continue
            try:
                samples.append(payload_from_line(line))
            except (json.JSONDecodeError, TypeError, ValueError) as exc:
                malformed_count += 1
                if malformed_count <= 5:
                    text = line.decode("utf-8", errors="replace").strip()
                    print(f"Ignoring non-IMU line: {text!r} ({exc})")

    if malformed_count > 5:
        print(f"Ignored {malformed_count} non-IMU lines total.")

    return samples


def norm3(x: float, y: float, z: float) -> float:
    return math.sqrt((x * x) + (y * y) + (z * z))


def mean(values: List[float]) -> float:
    return statistics.fmean(values) if values else float("nan")


def pstdev(values: List[float]) -> float:
    return statistics.pstdev(values) if len(values) > 1 else 0.0


def report(samples: List[Dict[str, float]], elapsed: float, args: argparse.Namespace) -> int:
    if not samples:
        print("FAIL: no valid IMU JSON samples were received.")
        return 1

    accel_norms = [
        norm3(sample["accel_x"], sample["accel_y"], sample["accel_z"])
        for sample in samples
    ]
    gyro_norms = [
        norm3(sample["gyro_x"], sample["gyro_y"], sample["gyro_z"])
        for sample in samples
    ]
    sample_rate = len(samples) / elapsed if elapsed > 0.0 else 0.0
    accel_norm_mean = mean(accel_norms)
    gyro_norm_mean = mean(gyro_norms)

    print(f"Samples: {len(samples)} in {elapsed:.2f}s ({sample_rate:.2f} Hz)")
    print(
        "Accel mean:"
        f" x={mean([s['accel_x'] for s in samples]): .3f}"
        f" y={mean([s['accel_y'] for s in samples]): .3f}"
        f" z={mean([s['accel_z'] for s in samples]): .3f} m/s^2"
    )
    print(
        "Gyro mean: "
        f" x={mean([s['gyro_x'] for s in samples]): .3f}"
        f" y={mean([s['gyro_y'] for s in samples]): .3f}"
        f" z={mean([s['gyro_z'] for s in samples]): .3f} rad/s"
    )
    print(
        f"Accel norm mean={accel_norm_mean:.3f} m/s^2 "
        f"stdev={pstdev(accel_norms):.3f}"
    )
    print(
        f"Gyro norm mean={gyro_norm_mean:.3f} rad/s "
        f"stdev={pstdev(gyro_norms):.3f}"
    )

    failures = []
    if sample_rate < args.min_rate or sample_rate > args.max_rate:
        failures.append(
            f"sample rate {sample_rate:.2f} Hz is outside "
            f"{args.min_rate:.2f}-{args.max_rate:.2f} Hz"
        )

    gravity_error = abs(accel_norm_mean - GRAVITY_M_S2)
    if gravity_error > args.gravity_tolerance:
        failures.append(
            f"accel norm is {gravity_error:.2f} m/s^2 away from gravity; "
            "keep the MPU still and check scale/orientation"
        )

    if gyro_norm_mean > args.stationary_gyro_limit:
        failures.append(
            f"gyro norm {gyro_norm_mean:.2f} rad/s is high for a stationary MPU"
        )

    if failures:
        print("FAIL:")
        for failure in failures:
            print(f"  - {failure}")
        return 1

    print("PASS: MPU stream looks sane for a stationary sensor.")
    return 0


def main() -> int:
    args = parse_args()
    print_available_ports()
    print(f"Reading {args.port} at {args.baud} baud for {args.seconds:.1f}s...")

    started_at = time.monotonic()
    try:
        samples = collect_samples(args)
    except serial.SerialException as exc:
        print(f"FAIL: could not open/read {args.port}: {exc}")
        return 1

    elapsed = time.monotonic() - started_at
    return report(samples, elapsed, args)


if __name__ == "__main__":
    raise SystemExit(main())
