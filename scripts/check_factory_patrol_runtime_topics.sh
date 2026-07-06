#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

safe_source_setup() {
  local setup_file="$1"
  echo "[factory-topic-check] Sourcing ${setup_file}"
  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
}

if [[ -f "install/setup.bash" ]]; then
  safe_source_setup "install/setup.bash"
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "FAIL: ros2 command not found. Source ROS2 first, for example:" >&2
  echo "  source /opt/ros/jazzy/setup.bash" >&2
  exit 1
fi

TOPICS=(
  "/clock"
  "/tf"
  "/joint_states"
  "/odom"
  "/scan"
  "/cmd_vel"
  "/mission_runner/state"
  "/safety_state"
  "/localization/health"
  "/amr_simulation/markers"
  "/amr_simulation/demo_timeline"
)

echo "[factory-topic-check] Checking Factory Patrol runtime topics..."
if ! topic_list="$(timeout 8s ros2 topic list 2>/dev/null)"; then
  echo "FAIL: unable to read ROS2 topic list within timeout" >&2
  exit 1
fi

failures=0
for topic in "${TOPICS[@]}"; do
  if grep -Fxq "${topic}" <<<"${topic_list}"; then
    echo "PASS: ${topic}"
  else
    echo "FAIL: missing ${topic}" >&2
    failures=$((failures + 1))
  fi
done

if [[ "${failures}" -eq 0 ]]; then
  echo "[factory-topic-check] PASS: all expected Factory Patrol topics are present."
  exit 0
fi

echo "[factory-topic-check] FAIL: ${failures} expected topic(s) missing." >&2
exit 1
