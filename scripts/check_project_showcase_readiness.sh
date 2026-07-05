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
  if [ -f "$path" ]; then
    pass "file exists: $path"
  else
    fail "missing file: $path"
  fi
}

check_dir() {
  local path="$1"
  if [ -d "$path" ]; then
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

echo "Checking Phase 6 project showcase readiness..."

check_file "README.md"
check_file "docs/roadmap.md"
check_file "docs/experiment_report.md"
check_file "docs/interview_notes.md"
check_file "docs/showcase/README.md"
check_file "scripts/README.md"
check_file ".github/workflows/ci.yml"

check_dir "docs/showcase"
check_dir "docs/showcase/figures"
check_dir "docs/showcase/screenshots"

check_file "scripts/check_nav2_costmap_obstacle_layer.sh"
check_file "scripts/run_nav2_basic_demo.sh"
check_file "scripts/check_nav2_runtime_topics.sh"
check_file "scripts/check_factory_patrol_assets.sh"
check_file "scripts/check_factory_patrol_demo_workflows.sh"
check_file "scripts/check_factory_patrol_demo_runtime.sh"
check_file "scripts/check_safety_state_machine.sh"

check_grep "Closed-Loop Pipeline" "README.md" "README contains closed-loop summary"
check_grep "Validation Scripts" "README.md" "README contains validation script section"
check_grep "Phase 6" "docs/roadmap.md" "roadmap contains Phase 6"
check_grep "TBD" "docs/experiment_report.md" "experiment report keeps TBD placeholders"
check_grep "Do not invent metrics" "docs/experiment_report.md" "experiment report states result policy"
check_grep "not yet validated" "docs/showcase/README.md" "showcase README avoids fake runtime claims"
check_grep "static-checks" ".github/workflows/ci.yml" "CI is configured for static checks"

if [ "$failures" -eq 0 ]; then
  echo "Project showcase readiness static check passed."
  exit 0
fi

echo "Project showcase readiness static check failed with $failures issue(s)."
exit 1
