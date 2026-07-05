#!/usr/bin/env bash
set -euo pipefail

if [[ "$#" -lt 1 ]]; then
  echo "Usage: bash scripts/run_tracking_analysis_workflow.sh <tracking.csv> [more_tracking.csv ...]" >&2
  exit 1
fi

if ! command -v python3 >/dev/null 2>&1; then
  echo "FAIL: python3 command not found" >&2
  exit 1
fi

OUTPUT_DIR="src/robot_experiments/results/figures"

for csv_file in "$@"; do
  if [[ ! -f "${csv_file}" ]]; then
    echo "FAIL: CSV file not found: ${csv_file}" >&2
    exit 1
  fi

  echo "[tracking-analysis] Analyzing ${csv_file}"
  python3 scripts/analyze_tracking_result.py "${csv_file}"
  echo
  echo "[tracking-analysis] Plotting ${csv_file} -> ${OUTPUT_DIR}"
  python3 scripts/plot_tracking_result.py "${csv_file}" --output-dir "${OUTPUT_DIR}"
  echo
done

if [[ "$#" -gt 1 ]]; then
  echo "[tracking-analysis] Multiple CSV files were provided. Markdown comparison:"
  python3 scripts/compare_tracking_results.py --format markdown "$@"
else
  echo "[tracking-analysis] To compare multiple controllers later, run:"
  echo "  python3 scripts/compare_tracking_results.py --format markdown <csv1> <csv2>"
fi
