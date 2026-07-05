#!/usr/bin/env python3
import argparse
import sys
from pathlib import Path

from tracking_analysis_utils import TrackingAnalysisError, numeric_series, read_tracking_csv


DEFAULT_OUTPUT_DIR = Path("src/robot_experiments/results/figures")


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Plot standalone path tracking CSV results.")
    parser.add_argument("csv_file", help="tracking CSV file")
    parser.add_argument(
        "--output-dir",
        default=str(DEFAULT_OUTPUT_DIR),
        help=f"directory for generated PNG files; default: {DEFAULT_OUTPUT_DIR}",
    )
    return parser.parse_args(argv)


def load_matplotlib():
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        return plt
    except ImportError:
        print(
            "ERROR: matplotlib is required for plotting. Install python3-matplotlib "
            "or run only analyze_tracking_result.py.",
            file=sys.stderr,
        )
        return None


def save_figure(plt, output_path):
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()
    print(f"wrote {output_path}")


def plot_trajectory(plt, rows, output_dir):
    x = numeric_series(rows, "x")
    y = numeric_series(rows, "y")
    ref_x = numeric_series(rows, "ref_x")
    ref_y = numeric_series(rows, "ref_y")

    plt.figure(figsize=(7, 6))
    plt.plot(ref_x, ref_y, "--", label="reference path")
    plt.plot(x, y, "-", label="actual trajectory")
    plt.scatter([x[0]], [y[0]], marker="o", label="start")
    plt.scatter([x[-1]], [y[-1]], marker="x", label="end")
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.title("Tracking Trajectory")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    save_figure(plt, output_dir / "trajectory.png")


def plot_lateral_error(plt, rows, output_dir):
    sample_index = numeric_series(rows, "sample_index")
    lateral_error = numeric_series(rows, "lateral_error")

    plt.figure(figsize=(8, 4))
    plt.plot(sample_index, lateral_error, label="lateral error")
    plt.xlabel("sample index")
    plt.ylabel("lateral error (m)")
    plt.title("Lateral Error")
    plt.grid(True)
    plt.legend()
    save_figure(plt, output_dir / "lateral_error.png")


def plot_heading_error(plt, rows, output_dir):
    sample_index = numeric_series(rows, "sample_index")
    heading_error = numeric_series(rows, "heading_error")

    plt.figure(figsize=(8, 4))
    plt.plot(sample_index, heading_error, label="heading error")
    plt.xlabel("sample index")
    plt.ylabel("heading error (rad)")
    plt.title("Heading Error")
    plt.grid(True)
    plt.legend()
    save_figure(plt, output_dir / "heading_error.png")


def plot_cmd_vel(plt, rows, output_dir):
    sample_index = numeric_series(rows, "sample_index")
    linear_velocity = numeric_series(rows, "linear_velocity")
    angular_velocity = numeric_series(rows, "angular_velocity")

    plt.figure(figsize=(8, 4))
    plt.plot(sample_index, linear_velocity, label="linear velocity (m/s)")
    plt.plot(sample_index, angular_velocity, label="angular velocity (rad/s)")
    plt.xlabel("sample index")
    plt.ylabel("command")
    plt.title("Command Velocity")
    plt.grid(True)
    plt.legend()
    save_figure(plt, output_dir / "cmd_vel.png")


def main(argv):
    args = parse_args(argv)
    plt = load_matplotlib()
    if plt is None:
        return 1

    try:
        rows = read_tracking_csv(args.csv_file)
        output_dir = Path(args.output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        plot_trajectory(plt, rows, output_dir)
        plot_lateral_error(plt, rows, output_dir)
        plot_heading_error(plt, rows, output_dir)
        plot_cmd_vel(plt, rows, output_dir)
    except (OSError, TrackingAnalysisError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
