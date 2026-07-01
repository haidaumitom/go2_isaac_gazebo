#!/usr/bin/env python3
"""Summarize left-right gait asymmetry from an rl_sim policy debug CSV."""

import argparse
import csv
import math
from pathlib import Path


JOINT_PAIRS = [
    ("FL_hip_joint", "FR_hip_joint"),
    ("FL_thigh_joint", "FR_thigh_joint"),
    ("FL_calf_joint", "FR_calf_joint"),
    ("RL_hip_joint", "RR_hip_joint"),
    ("RL_thigh_joint", "RR_thigh_joint"),
    ("RL_calf_joint", "RR_calf_joint"),
]

METRICS = ["action", "target_q", "actual_q", "error_q", "actual_dq", "target_tau"]


def parse_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def include_row(row, args):
    cmd_x = parse_float(row.get("cmd_x"))
    cmd_y = parse_float(row.get("cmd_y"))
    cmd_yaw = parse_float(row.get("cmd_yaw"))

    if args.min_abs_x is not None and abs(cmd_x) < args.min_abs_x:
        return False
    if args.max_abs_y is not None and abs(cmd_y) > args.max_abs_y:
        return False
    if args.max_abs_yaw is not None and abs(cmd_yaw) > args.max_abs_yaw:
        return False
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path", type=Path)
    parser.add_argument("--min-abs-x", type=float, default=0.3, help="Only analyze rows with |cmd_x| above this value.")
    parser.add_argument("--max-abs-y", type=float, default=0.05, help="Only analyze near-straight rows with |cmd_y| below this value.")
    parser.add_argument("--max-abs-yaw", type=float, default=0.05, help="Only analyze near-straight rows with |cmd_yaw| below this value.")
    args = parser.parse_args()

    rows = []
    with args.csv_path.open(newline="") as file:
        reader = csv.DictReader(file)
        for row in reader:
            if include_row(row, args):
                rows.append(row)

    if not rows:
        print("No rows matched the command filter.")
        return 1

    print(f"CSV: {args.csv_path}")
    print(f"Rows analyzed: {len(rows)}")
    print("Filter: |cmd_x| >= %.3f, |cmd_y| <= %.3f, |cmd_yaw| <= %.3f" % (
        args.min_abs_x,
        args.max_abs_y,
        args.max_abs_yaw,
    ))
    print()
    print("left_minus_right means positive = left side larger than right side")
    print()

    for metric in METRICS:
        print(metric)
        for left_joint, right_joint in JOINT_PAIRS:
            left_key = f"{metric}_{left_joint}"
            right_key = f"{metric}_{right_joint}"
            diffs = [
                parse_float(row.get(left_key)) - parse_float(row.get(right_key))
                for row in rows
                if left_key in row and right_key in row
            ]
            diffs = [value for value in diffs if math.isfinite(value)]
            if not diffs:
                print(f"  {left_joint} - {right_joint}: missing")
                continue
            mean = sum(diffs) / len(diffs)
            rms = math.sqrt(sum(value * value for value in diffs) / len(diffs))
            print(f"  {left_joint} - {right_joint}: mean={mean:+.5f} rms={rms:.5f}")
        print()


if __name__ == "__main__":
    raise SystemExit(main())
