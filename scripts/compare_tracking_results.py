#!/usr/bin/env python3
import argparse
import sys

from tracking_analysis_utils import (
    TrackingAnalysisError,
    compute_tracking_metrics,
    format_metric_value,
    read_tracking_csv,
)


FIELDS = [
    "controller",
    "path_name",
    "sample_count",
    "goal_reached",
    "rms_lateral_error",
    "max_lateral_error",
    "mean_abs_heading_error",
    "max_abs_heading_error",
    "mean_linear_velocity",
    "max_linear_velocity",
    "max_abs_angular_velocity",
    "final_distance_to_goal",
    "csv_file",
]


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Compare multiple standalone path tracking CSV results."
    )
    parser.add_argument("csv_files", nargs="+", help="tracking CSV files")
    parser.add_argument(
        "--format",
        choices=["text", "markdown"],
        default="text",
        help="output format",
    )
    return parser.parse_args(argv)


def load_metrics(csv_files):
    metrics = []
    for csv_file in csv_files:
        rows = read_tracking_csv(csv_file)
        metrics.append(compute_tracking_metrics(rows, csv_file=csv_file))
    return metrics


def print_text(metrics):
    for index, row in enumerate(metrics):
        if index:
            print()
        for field in FIELDS:
            print(f"{field}: {format_metric_value(row[field])}")


def print_markdown(metrics):
    print("| " + " | ".join(FIELDS) + " |")
    print("| " + " | ".join(["---"] * len(FIELDS)) + " |")
    for row in metrics:
        values = [format_metric_value(row[field]) for field in FIELDS]
        print("| " + " | ".join(values) + " |")


def main(argv):
    args = parse_args(argv)
    try:
        metrics = load_metrics(args.csv_files)
    except TrackingAnalysisError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    if args.format == "markdown":
        print_markdown(metrics)
    else:
        print_text(metrics)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
