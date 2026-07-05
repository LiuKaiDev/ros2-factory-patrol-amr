#!/usr/bin/env python3
import csv
import math
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


class TrackingAnalysisError(Exception):
    pass


def validate_required_fields(fieldnames):
    missing = [field for field in REQUIRED_FIELDS if field not in (fieldnames or [])]
    if missing:
        raise TrackingAnalysisError("CSV missing required fields: " + ", ".join(missing))


def read_tracking_csv(csv_file):
    csv_path = Path(csv_file)
    if not csv_path.is_file():
        raise TrackingAnalysisError(f"CSV file does not exist: {csv_path}")

    try:
        with csv_path.open(newline="", encoding="utf-8-sig") as handle:
            reader = csv.DictReader(handle)
            validate_required_fields(reader.fieldnames)
            rows = list(reader)
    except OSError as exc:
        raise TrackingAnalysisError(f"failed to read CSV: {exc}") from exc

    if not rows:
        raise TrackingAnalysisError("CSV contains no samples")
    return rows


def parse_float(row, field):
    try:
        return float(row[field])
    except ValueError as exc:
        raise TrackingAnalysisError(f"field '{field}' is not a float: {row[field]!r}") from exc


def parse_bool(value):
    return str(value).strip().lower() in {"1", "true", "yes"}


def mean(values):
    return sum(values) / len(values) if values else 0.0


def rms(values):
    return math.sqrt(sum(value * value for value in values) / len(values)) if values else 0.0


def numeric_series(rows, field):
    return [parse_float(row, field) for row in rows]


def compute_tracking_metrics(rows, csv_file=None):
    lateral_errors = numeric_series(rows, "lateral_error")
    heading_errors = numeric_series(rows, "heading_error")
    linear_velocities = numeric_series(rows, "linear_velocity")
    angular_velocities = numeric_series(rows, "angular_velocity")
    final_distance_to_goal = parse_float(rows[-1], "distance_to_goal")
    controllers = sorted({row["controller"] for row in rows})
    path_names = sorted({row["path_name"] for row in rows})
    goal_reached = any(parse_bool(row["goal_reached"]) for row in rows)

    metrics = {
        "controller": ",".join(controllers),
        "path_name": ",".join(path_names),
        "sample_count": len(rows),
        "goal_reached": goal_reached,
        "mean_lateral_error": mean(lateral_errors),
        "rms_lateral_error": rms(lateral_errors),
        "max_lateral_error": max(abs(value) for value in lateral_errors),
        "mean_abs_heading_error": mean([abs(value) for value in heading_errors]),
        "max_abs_heading_error": max(abs(value) for value in heading_errors),
        "mean_linear_velocity": mean(linear_velocities),
        "max_linear_velocity": max(linear_velocities),
        "max_abs_angular_velocity": max(abs(value) for value in angular_velocities),
        "final_distance_to_goal": final_distance_to_goal,
    }
    if csv_file is not None:
        metrics["csv_file"] = str(csv_file)
    return metrics


def format_metric_value(value):
    if isinstance(value, bool):
        return str(value).lower()
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return f"{value:.6f}"
    return str(value)
