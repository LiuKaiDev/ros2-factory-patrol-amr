#!/usr/bin/env python3
import sys

from tracking_analysis_utils import (
    TrackingAnalysisError,
    compute_tracking_metrics,
    format_metric_value,
    read_tracking_csv,
)


OUTPUT_FIELDS = [
    "controller",
    "path_name",
    "sample_count",
    "goal_reached",
    "mean_lateral_error",
    "rms_lateral_error",
    "max_lateral_error",
    "mean_abs_heading_error",
    "max_abs_heading_error",
    "mean_linear_velocity",
    "max_linear_velocity",
    "max_abs_angular_velocity",
    "final_distance_to_goal",
]


def fail(message):
    print(f"ERROR: {message}", file=sys.stderr)
    return 1


def main(argv):
    if len(argv) != 2:
        return fail("usage: python3 scripts/analyze_tracking_result.py <csv_file>")

    try:
        rows = read_tracking_csv(argv[1])
        metrics = compute_tracking_metrics(rows)
    except TrackingAnalysisError as exc:
        return fail(str(exc))

    for key in OUTPUT_FIELDS:
        print(f"{key}: {format_metric_value(metrics[key])}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
