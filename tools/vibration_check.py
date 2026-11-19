#!/usr/bin/env python3
"""
vibration_check.py - Analyze vibration levels in ArduPilot dataflash logs.

Reads VIBE messages from a .bin (dataflash) or .tlog (telemetry) log file and
reports per-axis vibration statistics together with clipping event counts.
Exits with a non-zero status if any threshold is exceeded so the script can
be used in CI / preflight automation.

Usage:
    python vibration_check.py <logfile> [options]

Examples:
    python vibration_check.py flight.bin
    python vibration_check.py flight.bin --warn 30 --fail 60
    python vibration_check.py flight.bin --csv --output report.csv

Requirements:
    pip install pymavlink
"""

import argparse
import csv
import sys
from pathlib import Path

try:
    from pymavlink import mavutil
except ImportError:
    print("ERROR: pymavlink is not installed. Run: pip install pymavlink", file=sys.stderr)
    sys.exit(2)

# ArduPilot recommended vibration limits (m/s^2)
DEFAULT_WARN_MS2 = 30.0   # yellow: investigate
DEFAULT_FAIL_MS2 = 60.0   # red: unsafe to fly

# Clipping: even a handful of events per second indicates IMU saturation
DEFAULT_CLIP_WARN = 100
DEFAULT_CLIP_FAIL = 1000


def parse_args():
    parser = argparse.ArgumentParser(
        description="Check vibration levels in an ArduPilot log file."
    )
    parser.add_argument("logfile", help="Path to .bin or .tlog log file")
    parser.add_argument(
        "--warn",
        type=float,
        default=DEFAULT_WARN_MS2,
        metavar="MS2",
        help=f"Warn threshold in m/s^2 (default: {DEFAULT_WARN_MS2})",
    )
    parser.add_argument(
        "--fail",
        type=float,
        default=DEFAULT_FAIL_MS2,
        metavar="MS2",
        help=f"Fail threshold in m/s^2 (default: {DEFAULT_FAIL_MS2})",
    )
    parser.add_argument(
        "--clip-warn",
        type=int,
        default=DEFAULT_CLIP_WARN,
        dest="clip_warn",
        help=f"Warn threshold for total clip events (default: {DEFAULT_CLIP_WARN})",
    )
    parser.add_argument(
        "--clip-fail",
        type=int,
        default=DEFAULT_CLIP_FAIL,
        dest="clip_fail",
        help=f"Fail threshold for total clip events (default: {DEFAULT_CLIP_FAIL})",
    )
    parser.add_argument(
        "--csv",
        action="store_true",
        help="Write per-sample data to a CSV file",
    )
    parser.add_argument(
        "--output",
        metavar="FILE",
        default=None,
        help="CSV output path (default: <logfile>.vibe.csv)",
    )
    parser.add_argument(
        "--quiet",
        "-q",
        action="store_true",
        help="Suppress informational output; only print warnings and errors",
    )
    return parser.parse_args()


class VibeSample:
    """Single VIBE message sample."""

    __slots__ = ("time_us", "vibe_x", "vibe_y", "vibe_z", "clip0", "clip1", "clip2")

    def __init__(self, msg):
        self.time_us = getattr(msg, "TimeUS", 0)
        self.vibe_x = float(getattr(msg, "VibeX", 0.0))
        self.vibe_y = float(getattr(msg, "VibeY", 0.0))
        self.vibe_z = float(getattr(msg, "VibeZ", 0.0))
        self.clip0 = int(getattr(msg, "Clip0", 0))
        self.clip1 = int(getattr(msg, "Clip1", 0))
        self.clip2 = int(getattr(msg, "Clip2", 0))


def load_vibe_messages(logfile: str) -> list:
    """Return a list of VibeSample objects parsed from *logfile*."""
    mlog = mavutil.mavlink_connection(logfile, robust_parsing=True)
    samples = []
    while True:
        msg = mlog.recv_match(type="VIBE", blocking=False)
        if msg is None:
            break
        samples.append(VibeSample(msg))
    return samples


def compute_stats(samples: list) -> dict:
    """Compute per-axis min/max/mean and total clip counts from samples."""
    if not samples:
        return {}

    axes = {"x": [], "y": [], "z": []}
    clips = [0, 0, 0]

    for s in samples:
        axes["x"].append(s.vibe_x)
        axes["y"].append(s.vibe_y)
        axes["z"].append(s.vibe_z)
        clips[0] = max(clips[0], s.clip0)
        clips[1] = max(clips[1], s.clip1)
        clips[2] = max(clips[2], s.clip2)

    stats = {}
    for axis, values in axes.items():
        stats[axis] = {
            "min": min(values),
            "max": max(values),
            "mean": sum(values) / len(values),
            "samples": len(values),
        }

    # Clip counters in VIBE are cumulative totals, so final value = total events
    stats["clips"] = {
        "imu0": clips[0],
        "imu1": clips[1],
        "imu2": clips[2],
        "total": sum(clips),
    }

    return stats


def severity_label(value: float, warn: float, fail: float) -> str:
    if value >= fail:
        return "FAIL"
    if value >= warn:
        return "WARN"
    return "OK"


def clip_severity_label(count: int, warn: int, fail: int) -> str:
    if count >= fail:
        return "FAIL"
    if count >= warn:
        return "WARN"
    return "OK"


def print_report(stats: dict, args, sample_count: int) -> int:
    """Print human-readable report. Returns exit code (0=ok, 1=warn, 2=fail)."""
    overall = 0

    print(f"\n{'='*60}")
    print(f" Vibration Report")
    print(f"{'='*60}")
    print(f" Samples analysed : {sample_count}")
    print(f" Warn threshold   : {args.warn} m/s^2")
    print(f" Fail threshold   : {args.fail} m/s^2")
    print(f"{'='*60}")
    print(f" {'Axis':<8} {'Min':>8} {'Mean':>8} {'Max':>8}   Status")
    print(f" {'-'*8} {'-'*8} {'-'*8} {'-'*8}   {'-'*6}")

    for axis in ("x", "y", "z"):
        s = stats[axis]
        label = severity_label(s["max"], args.warn, args.fail)
        if label == "FAIL":
            overall = max(overall, 2)
        elif label == "WARN":
            overall = max(overall, 1)
        print(
            f" {'Vibe' + axis.upper():<8} {s['min']:>8.2f} {s['mean']:>8.2f}"
            f" {s['max']:>8.2f}   {label}"
        )

    print(f"\n{'='*60}")
    print(f" IMU Clipping Events")
    print(f"{'='*60}")
    clips = stats["clips"]
    for imu_key in ("imu0", "imu1", "imu2"):
        count = clips[imu_key]
        label = clip_severity_label(count, args.clip_warn, args.clip_fail)
        if label == "FAIL":
            overall = max(overall, 2)
        elif label == "WARN":
            overall = max(overall, 1)
        print(f" {imu_key.upper():<8} {count:>8} events   {label}")

    total_clips = clips["total"]
    print(f"\n Total clips: {total_clips}")
    print(f"{'='*60}")

    result_label = {0: "PASS", 1: "WARNING", 2: "FAIL"}[overall]
    print(f"\n Overall result: {result_label}\n")

    return overall


def write_csv(samples: list, output_path: str) -> None:
    """Write per-sample vibration data to a CSV file."""
    with open(output_path, "w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(["TimeUS", "VibeX", "VibeY", "VibeZ", "Clip0", "Clip1", "Clip2"])
        for s in samples:
            writer.writerow([
                s.time_us, s.vibe_x, s.vibe_y, s.vibe_z,
                s.clip0, s.clip1, s.clip2,
            ])


def main() -> int:
    args = parse_args()

    logpath = Path(args.logfile)
    if not logpath.exists():
        print(f"ERROR: File not found: {args.logfile}", file=sys.stderr)
        return 2

    if not args.quiet:
        print(f"Loading VIBE messages from: {logpath}")

    try:
        samples = load_vibe_messages(str(logpath))
    except Exception as exc:
        print(f"ERROR: Failed to parse log: {exc}", file=sys.stderr)
        return 2

    if not samples:
        print(
            "WARNING: No VIBE messages found in log. "
            "The vehicle may not have been armed, or this log format does not "
            "contain VIBE data.",
            file=sys.stderr,
        )
        return 1

    if not args.quiet:
        print(f"Found {len(samples)} VIBE samples.")

    stats = compute_stats(samples)

    if args.csv:
        output_path = args.output or (str(logpath) + ".vibe.csv")
        write_csv(samples, output_path)
        if not args.quiet:
            print(f"CSV written to: {output_path}")

    exit_code = print_report(stats, args, len(samples))
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
