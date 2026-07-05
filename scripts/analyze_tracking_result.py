#!/usr/bin/env python3
import csv
import math
import sys
from pathlib import Path


REQUIRED_FIELDS = [
    "timestamp_sec",
    "controller",
    "path_name",
    "sample_index",
    "x",
    "y",
    "yaw",
    "ref_x",
    "ref_y",
    "ref_yaw",
    "linear_velocity",
    "angular_velocity",
    "lateral_error",
    "heading_error",
    "distance_to_goal",
    "goal_reached",
]


def fail(message: str) -> int:
    print(f"ERROR: {message}", file=sys.stderr)
    return 1


def parse_float(row, field):
    try:
        return float(row[field])
    except ValueError as exc:
        raise ValueError(f"field '{field}' is not a float: {row[field]!r}") from exc


def mean(values):
    return sum(values) / len(values) if values else 0.0


def rms(values):
    return math.sqrt(sum(value * value for value in values) / len(values)) if values else 0.0


def main(argv):
    if len(argv) != 2:
        return fail("usage: python3 scripts/analyze_tracking_result.py <csv_file>")

    csv_path = Path(argv[1])
    if not csv_path.is_file():
        return fail(f"CSV file does not exist: {csv_path}")

    try:
        with csv_path.open(newline="", encoding="utf-8-sig") as handle:
            reader = csv.DictReader(handle)
            missing = [field for field in REQUIRED_FIELDS if field not in (reader.fieldnames or [])]
            if missing:
                return fail("CSV missing required fields: " + ", ".join(missing))
            rows = list(reader)
    except OSError as exc:
        return fail(f"failed to read CSV: {exc}")

    if not rows:
        return fail("CSV contains no samples")

    try:
        lateral_errors = [parse_float(row, "lateral_error") for row in rows]
        heading_errors = [parse_float(row, "heading_error") for row in rows]
        linear_velocities = [parse_float(row, "linear_velocity") for row in rows]
        angular_velocities = [parse_float(row, "angular_velocity") for row in rows]
        final_distance_to_goal = parse_float(rows[-1], "distance_to_goal")
    except ValueError as exc:
        return fail(str(exc))

    controllers = sorted({row["controller"] for row in rows})
    path_names = sorted({row["path_name"] for row in rows})
    goal_reached = any(row["goal_reached"].strip().lower() == "true" for row in rows)

    results = {
        "controller": ",".join(controllers),
        "path_name": ",".join(path_names),
        "sample_count": str(len(rows)),
        "goal_reached": str(goal_reached).lower(),
        "mean_lateral_error": f"{mean(lateral_errors):.6f}",
        "rms_lateral_error": f"{rms(lateral_errors):.6f}",
        "max_lateral_error": f"{max(abs(value) for value in lateral_errors):.6f}",
        "mean_abs_heading_error": f"{mean([abs(value) for value in heading_errors]):.6f}",
        "max_abs_heading_error": f"{max(abs(value) for value in heading_errors):.6f}",
        "mean_linear_velocity": f"{mean(linear_velocities):.6f}",
        "max_linear_velocity": f"{max(linear_velocities):.6f}",
        "max_abs_angular_velocity": f"{max(abs(value) for value in angular_velocities):.6f}",
        "final_distance_to_goal": f"{final_distance_to_goal:.6f}",
    }

    for key, value in results.items():
        print(f"{key}: {value}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
