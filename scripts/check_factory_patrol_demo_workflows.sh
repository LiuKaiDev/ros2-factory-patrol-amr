#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

fail() {
  echo "FAIL: $*" >&2
  exit 1
}

pass() {
  echo "PASS: $*"
}

require_file() {
  local file="$1"
  [[ -f "${ROOT_DIR}/${file}" ]] || fail "missing file: ${file}"
  pass "found ${file}"
}

require_grep() {
  local pattern="$1"
  local file="$2"
  local message="$3"
  grep -q "${pattern}" "${ROOT_DIR}/${file}" || fail "${message}"
  pass "${file} contains ${pattern}"
}

FILES=(
  "scripts/run_factory_patrol_multipoint_demo.sh"
  "scripts/print_factory_patrol_goals.py"
  "src/robot_simulation/config/factory_patrol_multipoint_mission.yaml"
  "src/robot_simulation/config/factory_patrol_obstacle_demo.yaml"
  "src/robot_simulation/models/temporary_box_obstacle/model.sdf"
  "src/robot_simulation/models/temporary_box_obstacle/model.config"
  "scripts/run_factory_patrol_obstacle_demo.sh"
  "src/robot_simulation/config/factory_patrol_localization_recovery.yaml"
  "scripts/run_factory_patrol_localization_recovery_demo.sh"
  "scripts/check_factory_patrol_demo_runtime.sh"
  "docs/simulation_scenarios.md"
  "docs/experiment_report.md"
  "docs/localization.md"
  "docs/safety_state_machine.md"
  "README.md"
)

for file in "${FILES[@]}"; do
  require_file "${file}"
done

require_grep "station_A" "scripts/run_factory_patrol_multipoint_demo.sh" \
  "multipoint demo script does not describe station_A"
require_grep "dry-run" "scripts/print_factory_patrol_goals.py" \
  "goal printer should default to dry-run"
require_grep "factory_patrol_multipoint_demo" \
  "src/robot_simulation/config/factory_patrol_multipoint_mission.yaml" \
  "multipoint mission profile is missing"
require_grep "temporary_box_obstacle" \
  "src/robot_simulation/config/factory_patrol_obstacle_demo.yaml" \
  "obstacle demo config is missing temporary obstacle"
require_grep "ros_gz_sim create" "scripts/run_factory_patrol_obstacle_demo.sh" \
  "obstacle demo script does not show spawn command"
require_grep "LOCALIZATION_LOST" \
  "src/robot_simulation/config/factory_patrol_localization_recovery.yaml" \
  "localization recovery config is missing expected LOST state"
require_grep "/initialpose" "scripts/run_factory_patrol_localization_recovery_demo.sh" \
  "localization recovery script does not show /initialpose injection"
require_grep "/localization/health" "scripts/check_factory_patrol_demo_runtime.sh" \
  "runtime topic check does not include localization health"
require_grep "/safety/state" "scripts/check_factory_patrol_demo_runtime.sh" \
  "runtime topic check does not include safety state"
require_grep "Phase 5B" "docs/simulation_scenarios.md" \
  "simulation docs do not mention Phase 5B"
require_grep "Factory Patrol Demo Acceptance" "docs/experiment_report.md" \
  "experiment report does not include factory demo acceptance template"
require_grep "run_factory_patrol_localization_recovery_demo.sh" "docs/localization.md" \
  "localization docs do not reference factory recovery demo"
require_grep "/safety/state" "docs/safety_state_machine.md" \
  "safety docs do not reference safety state"
require_grep "Phase 5B" "README.md" \
  "README does not mention Phase 5B demo workflows"

pass "factory patrol Phase 5B demo workflow scripts, configs, runtime check, docs, and templates are present"
