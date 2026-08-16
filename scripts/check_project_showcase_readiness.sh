#!/usr/bin/env bash
set -u

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}" || exit 1

failures=0

pass() { echo "PASS: $1"; }
fail() { echo "FAIL: $1"; failures=$((failures + 1)); }

check_file() {
  local path="$1"
  [[ -f "${path}" ]] && pass "file exists: ${path}" || fail "missing file: ${path}"
}

check_absent_path() {
  local path="$1"
  [[ ! -e "${path}" ]] && pass "development-only path is absent: ${path}" ||
    fail "development-only path remains: ${path}"
}

check_grep() {
  local pattern="$1"
  local path="$2"
  local label="$3"
  grep -Eq -- "${pattern}" "${path}" && pass "${label}" || fail "${label}"
}

check_public_docs_hygiene() {
  local -a markdown_files=()
  mapfile -t markdown_files < <(
    find . -type f -name '*.md' \
      -not -path './.git/*' \
      -not -path './build/*' \
      -not -path './install/*' \
      -not -path './log/*' | sort
  )

  if rg -n '(/home/|/mnt/|file://|chatgpt\.com|https?://(localhost|127\.0\.0\.1)|[A-Za-z]:\\\\)' \
      "${markdown_files[@]}"; then
    fail "public Markdown contains machine-specific or temporary links"
  else
    pass "public Markdown contains no machine-specific or temporary links"
  fi

  if rg -n -i 'compatibility marker|resume-ready|interview-ready|portfolio screenshot approval' \
      "${markdown_files[@]}"; then
    fail "public Markdown contains development workflow scaffolding"
  else
    pass "public Markdown is free of development workflow scaffolding"
  fi

  if python3 - "${markdown_files[@]}" <<'PY'
from pathlib import Path
from urllib.parse import unquote
import re
import sys

root = Path.cwd().resolve()
pattern = re.compile(r"!?\[[^]]*\]\(([^)]+)\)")
errors = []

for name in sys.argv[1:]:
    document = Path(name).resolve()
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
            if root not in (candidate, *candidate.parents):
                errors.append(f"{name}:{line_number}: link escapes repository: {target}")
            elif not candidate.exists():
                errors.append(f"{name}:{line_number}: missing local link target: {target}")

if errors:
    print("\n".join(errors), file=sys.stderr)
    raise SystemExit(1)
PY
  then
    pass "local Markdown links resolve"
  else
    fail "public Markdown contains broken local links"
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
    pass "committed benchmark JSON and CSV artifacts are consistent"
  else
    fail "committed benchmark JSON and CSV artifacts are inconsistent"
  fi
}

echo "Checking project repository consistency..."

for file in \
  LICENSE \
  README.md \
  docs/README.md \
  docs/demo.md \
  docs/architecture.md \
  docs/localization.md \
  docs/navigation.md \
  docs/control.md \
  docs/safety_state_machine.md \
  docs/experiment_report.md \
  docs/project_summary.md \
  docs/roadmap.md \
  scripts/README.md \
  .github/workflows/ci.yml \
  scripts/run_factory_patrol_demo.sh \
  scripts/run_factory_patrol_benchmarks.sh \
  scripts/check_factory_patrol_assets.sh \
  scripts/check_robot_interfaces_split.sh \
  src/robot_bringup/launch/factory_patrol_demo.launch.py \
  src/robot_perception/include/robot_perception/depth_projector.hpp \
  src/robot_perception/include/robot_perception/target_manager.hpp \
  src/robot_interfaces_perception/msg/DetectedObject3D.msg \
  src/robot_interfaces_perception/msg/PerceptionEvent.msg \
  src/robot_interfaces_perception/msg/PerceptionSafetyEvent.msg \
  src/robot_experiments/results/factory_patrol_phase8_20260815_011022.json \
  src/robot_experiments/results/factory_patrol_phase8_20260815_011022.csv; do
  check_file "${file}"
done

check_absent_path "AGENTS.md"
check_absent_path "docs/interview_notes.md"
check_absent_path "docs/upgrade"

check_grep '^# ROS2 工厂巡检 AMR 自主导航与巡检系统' README.md \
  "README identifies the complete Factory Patrol AMR"
check_grep 'Nav2' README.md "README documents Nav2 navigation"
check_grep 'AMCL' README.md "README documents localization"
check_grep 'Safety Gate' README.md "README documents final velocity authority"
check_grep 'Perception.*(/cmd_vel|速度)' README.md \
  "README documents the perception velocity boundary"
check_grep 'run_factory_patrol_demo.sh' docs/demo.md \
  "demo guide references the launch helper"
check_grep '--visual-inspection' docs/demo.md \
  "demo guide documents visual inspection mode"
check_grep '--perception-safety' docs/demo.md \
  "demo guide documents perception safety mode"
check_grep '--perception-diagnostics' docs/demo.md \
  "demo guide documents diagnostics mode"
check_grep 'run_factory_patrol_benchmarks.sh' docs/experiment_report.md \
  "experiment report documents benchmark reproduction"
check_grep 'nearest rank' docs/experiment_report.md \
  "experiment report defines percentile policy"
check_grep 'cmd_vel_safety_gate' docs/architecture.md \
  "architecture documentation names the final Safety Gate node"
check_grep 'check_robot_interfaces_split.sh' .github/workflows/ci.yml \
  "CI runs the interface architecture check"

check_public_docs_hygiene
check_benchmark_artifacts

if [[ "${failures}" -eq 0 ]]; then
  echo "Project repository consistency check passed."
  exit 0
fi

echo "Project repository consistency check failed with ${failures} issue(s)."
exit 1
