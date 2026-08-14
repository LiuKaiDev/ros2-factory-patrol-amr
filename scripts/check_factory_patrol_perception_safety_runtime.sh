#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

safe_source_setup() {
  local setup_file="$1"
  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
}

if [[ -f install/setup.bash ]]; then
  safe_source_setup install/setup.bash
fi

expected_type="robot_interfaces_perception/msg/PerceptionSafetyEvent"
actual_type="$(timeout 5s ros2 topic type /perception/safety_event 2>/dev/null || true)"
[[ "${actual_type}" == "${expected_type}" ]] || {
  echo "FAIL: /perception/safety_event expected ${expected_type}, got ${actual_type:-none}" >&2
  exit 1
}
echo "PASS: /perception/safety_event type=${actual_type}"

for node in /detector_node /geometry_validation_node; do
  publishers="$(timeout 5s ros2 node info "${node}" 2>/dev/null | \
    awk '/Publishers:/,/Service Servers:/' | \
    grep -E '/(cmd_vel|nav2_cmd_vel):' || true)"
  [[ -z "${publishers}" ]] || {
    echo "FAIL: perception node ${node} publishes velocity: ${publishers}" >&2
    exit 1
  }
done
echo "PASS: perception nodes publish no /cmd_vel or /nav2_cmd_vel"

timeout 300s python3 scripts/perception_safety_runtime_probe.py \
  --ros-args -p use_sim_time:=true
