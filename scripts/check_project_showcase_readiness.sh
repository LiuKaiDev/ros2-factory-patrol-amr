#!/usr/bin/env bash
set -u

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR" || exit 1

failures=0

pass() {
  echo "PASS: $1"
}

fail() {
  echo "FAIL: $1"
  failures=$((failures + 1))
}

check_file() {
  local path="$1"
  if [[ -f "$path" ]]; then
    pass "file exists: $path"
  else
    fail "missing file: $path"
  fi
}

check_dir() {
  local path="$1"
  if [[ -d "$path" ]]; then
    pass "directory exists: $path"
  else
    fail "missing directory: $path"
  fi
}

check_grep() {
  local pattern="$1"
  local path="$2"
  local label="$3"
  if grep -Eq "$pattern" "$path"; then
    pass "$label"
  else
    fail "$label"
  fi
}

check_absent() {
  local pattern="$1"
  local path="$2"
  local label="$3"
  if grep -Eq "$pattern" "$path"; then
    fail "$label"
  else
    pass "$label"
  fi
}

check_markdown_links() {
  if python3 - "$@" <<'PY'
from pathlib import Path
from urllib.parse import unquote
import re
import sys

root = Path.cwd().resolve()
pattern = re.compile(r"!?\[[^]]*\]\(([^)]+)\)")
errors = []

for name in sys.argv[1:]:
    document = (root / name).resolve()
    for line_number, line in enumerate(document.read_text(encoding="utf-8").splitlines(), 1):
        for match in pattern.finditer(line):
            target = match.group(1).strip()
            if target.startswith("<") and target.endswith(">"):
                target = target[1:-1]
            target = target.split(maxsplit=1)[0]
            if not target or target.startswith(("#", "http://", "https://", "mailto:")):
                continue
            path_text = unquote(target.split("#", 1)[0])
            candidate = (document.parent / path_text).resolve()
            if not candidate.exists():
                errors.append(f"{name}:{line_number}: missing local link target {target}")

if errors:
    print("\n".join(errors), file=sys.stderr)
    raise SystemExit(1)
PY
  then
    pass "README and final documentation local links resolve"
  else
    fail "README or final documentation contains a broken local link"
  fi
}

check_benchmark_artifacts() {
  if python3 - <<'PY'
import csv
import json
import math
from pathlib import Path

json_path = Path("src/robot_experiments/results/factory_patrol_phase8_20260815_011022.json")
csv_path = Path("src/robot_experiments/results/factory_patrol_phase8_20260815_011022.csv")
data = json.loads(json_path.read_text(encoding="utf-8"))
rows = {row["metric"]: row for row in csv.DictReader(csv_path.open(encoding="utf-8"))}

assert data["valid"] is True
assert data["metadata"]["detector"]["expected_device"] == "cpu"
assert data["metadata"]["detector"]["backend"] == "opencv_yolox"
assert data["mission"]["successes"] == 5
assert data["mission"]["failures"] == 0
assert data["false_task_triggers"]["false_triggers"] == 0
assert data["invalid_depth"]["metrics"]["correctly_rejected"] == 20
assert data["invalid_depth"]["metrics"]["false_valid_outputs"] == 0

sources = {
    "detector_processing_latency": data["detector"]["metrics"]["detector_processing_ms"],
    "localization_error": data["geometry"]["metrics"]["localization_error_m"],
    "detection_to_confirmation": data["detection_to_action"]["detection_to_confirmation_sec"],
    "confirmation_to_event": data["detection_to_action"]["confirmation_to_event_sec"],
    "event_to_nav2_goal": data["detection_to_action"]["event_to_nav2_goal_sec"],
    "detection_to_nav2_goal": data["detection_to_action"]["detection_to_nav2_goal_sec"],
    "safety_stop_response": data["safety"]["metrics"]["stop_response_sec"],
}

for name, source in sources.items():
    row = rows[name]
    assert int(row["sample_count"]) == source["count"]
    for field in ("mean", "p50", "p95", "max"):
        assert math.isclose(float(row[field]), float(source[field]), rel_tol=0.0, abs_tol=1e-12)
PY
  then
    pass "committed Phase 8 JSON and CSV artifacts are consistent"
  else
    fail "committed Phase 8 JSON and CSV artifacts are inconsistent"
  fi
}

echo "Checking Phase 9 project showcase readiness..."

for file in \
  README.md \
  docs/architecture.md \
  docs/roadmap.md \
  docs/experiment_report.md \
  docs/project_summary.md \
  docs/interview_notes.md \
  docs/showcase/README.md \
  scripts/README.md \
  .github/workflows/ci.yml \
  scripts/check_nav2_costmap_obstacle_layer.sh \
  scripts/run_nav2_basic_demo.sh \
  scripts/check_nav2_runtime_topics.sh \
  scripts/check_factory_patrol_assets.sh \
  scripts/check_factory_patrol_demo_workflows.sh \
  scripts/check_factory_patrol_demo_runtime.sh \
  scripts/check_safety_state_machine.sh \
  scripts/run_factory_patrol_benchmarks.sh \
  src/robot_experiments/results/factory_patrol_phase8_20260815_011022.json \
  src/robot_experiments/results/factory_patrol_phase8_20260815_011022.csv; do
  check_file "$file"
done

check_dir "docs/showcase"
check_dir "docs/showcase/figures"
check_dir "docs/showcase/screenshots"

check_grep "ROS2 Factory Patrol AMR with Visual Perception, Navigation and Safety Integration" \
  README.md "README contains final project positioning"
check_grep "Closed-Loop Pipeline" README.md "README contains the closed-loop architecture"
check_grep "Gazebo / WSL Simulation Benchmark" README.md "README labels simulation benchmark evidence"
check_grep "Perception never publishes" README.md "README states the perception velocity boundary"
check_grep "526\.189 ms" README.md "README contains canonical detector mean"
check_grep "0\.02351 m" README.md "README contains canonical localization RMSE"
check_grep "0\.214 s" README.md "README contains canonical STOP P95"
check_grep "648 tests" README.md "README contains the latest full test baseline"
check_grep "Known Limitations" README.md "README contains explicit limitations"
check_grep "Resume-ready Verified Metrics" docs/project_summary.md \
  "project summary contains verified resume metrics"
check_grep "two IDs" docs/project_summary.md "project summary documents target-ID limitation"
check_grep "Thirteen transient" docs/project_summary.md "project summary documents TF transients"
check_grep "Phase 9.*complete" docs/roadmap.md "roadmap records Phase 9 completion"
check_grep "run_factory_patrol_benchmarks.sh" docs/experiment_report.md \
  "experiment report documents benchmark reproduction"
check_grep "nearest rank" docs/experiment_report.md \
  "experiment report defines percentile policy"
check_grep "not yet validated" docs/showcase/README.md \
  "showcase README avoids fake media claims"
check_grep "static-checks" .github/workflows/ci.yml "CI is configured for static checks"
check_absent "https://chatgpt\.com/c/" README.md "README has no editor-session links"

check_markdown_links \
  README.md \
  docs/architecture.md \
  docs/experiment_report.md \
  docs/project_summary.md \
  docs/roadmap.md \
  docs/showcase/README.md \
  scripts/README.md
check_benchmark_artifacts

if [[ "$failures" -eq 0 ]]; then
  echo "Project showcase readiness static check passed."
  exit 0
fi

echo "Project showcase readiness static check failed with $failures issue(s)."
exit 1
