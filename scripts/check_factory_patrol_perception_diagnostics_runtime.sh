#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  # shellcheck disable=SC1091
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
fi
if [[ -f install/setup.bash ]]; then
  # shellcheck disable=SC1091
  set +u
  source install/setup.bash
  set -u
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "FAIL: ros2 command not found" >&2
  exit 1
fi

diag_pid=""
monitor_pid=""
fault_pid=""
detector_pid=""
cleanup() {
  for pid in "${detector_pid}" "${fault_pid}" "${monitor_pid}" "${diag_pid}"; do
    [[ -n "${pid}" ]] && kill "${pid}" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT

diag_executable="$(ros2 pkg prefix robot_perception)/lib/robot_perception/perception_diagnostics_node"
monitor_executable="$(ros2 pkg prefix robot_utils)/lib/robot_utils/system_monitor_node"
fault_executable="$(ros2 pkg prefix robot_utils)/lib/robot_utils/fault_supervisor_node"
detector_executable="$(ros2 pkg prefix robot_perception)/lib/robot_perception/detector_node"

"${diag_executable}" --ros-args \
  -p use_sim_time:=false -p detector_required:=false -p target_frame:=map \
  > /tmp/factory_patrol_perception_diagnostics.log 2>&1 &
diag_pid=$!
"${monitor_executable}" --ros-args \
  -p use_sim_time:=false -p monitor_perception:=true \
  > /tmp/factory_patrol_system_monitor_phase7.log 2>&1 &
monitor_pid=$!
"${fault_executable}" --ros-args \
  -p use_sim_time:=false -p startup_grace_ms:=1000 \
  > /tmp/factory_patrol_fault_supervisor_phase7.log 2>&1 &
fault_pid=$!

timeout 180s python3 scripts/perception_diagnostics_runtime_probe.py

"${detector_executable}" --ros-args -r __node:=phase7_detector_failure_probe \
  -p model_path:=/tmp/phase7_missing_detector_model.onnx \
  -p image_topic:=/phase7/missing_rgb -p debug_image_enabled:=false \
  > /tmp/factory_patrol_detector_failure_phase7.log 2>&1 &
detector_pid=$!
if timeout 12s ros2 topic echo --once --timeout 10 /perception/diagnostics \
  --filter "any(s.name == 'perception/detector' and s.hardware_id == 'factory_patrol_detector' and s.message == 'model/backend unavailable' for s in m.status)" \
  --no-arr >/dev/null 2>&1; then
  echo "PASS: actual detector model-load failure publishes ERROR diagnostics"
else
  echo "FAIL: detector model-load failure diagnostics were not observed" >&2
  exit 1
fi
kill "${detector_pid}" 2>/dev/null || true
wait "${detector_pid}" 2>/dev/null || true
detector_pid=""

node_publishers="$(timeout 5s ros2 node info /perception_diagnostics_node 2>/dev/null | \
  awk '/Publishers:/,/Service Servers:/')"
if grep -E '/(cmd_vel|nav2_cmd_vel):' <<<"${node_publishers}"; then
  echo "FAIL: perception diagnostics node exposes a velocity publisher" >&2
  exit 1
fi
echo "PASS: perception diagnostics node publishes no velocity command"
if grep -q '/perception/safety_event:' <<<"${node_publishers}"; then
  echo "FAIL: perception diagnostics node publishes a safety event" >&2
  exit 1
fi
echo "PASS: perception diagnostics node emits no synthetic safety CLEAR event"
