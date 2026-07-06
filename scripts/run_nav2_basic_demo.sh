#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${REPO_ROOT}"

PARAMS_FILE="src/robot_navigation/config/nav2_basic.yaml"
MAP_FILE="src/robot_navigation/maps/indoor_room.yaml"
RVIZ_FILE="src/robot_simulation/rviz/nav2_basic_debug.rviz"
PARAMS_PATH="${REPO_ROOT}/${PARAMS_FILE}"
MAP_PATH="${REPO_ROOT}/${MAP_FILE}"
RVIZ_PATH="${REPO_ROOT}/${RVIZ_FILE}"

safe_source_setup() {
  local setup_file="$1"
  echo "[nav2-basic-demo] Sourcing ${setup_file}"
  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
}

echo "[nav2-basic-demo] Repository: ${REPO_ROOT}"

[[ -f "${PARAMS_FILE}" ]] || {
  echo "FAIL: missing ${PARAMS_FILE}" >&2
  exit 1
}
[[ -f "${MAP_FILE}" ]] || {
  echo "FAIL: missing ${MAP_FILE}" >&2
  exit 1
}
[[ -f "${RVIZ_FILE}" ]] || {
  echo "FAIL: missing ${RVIZ_FILE}" >&2
  exit 1
}

if [[ -f "install/setup.bash" ]]; then
  safe_source_setup "install/setup.bash"
else
  echo "[nav2-basic-demo] install/setup.bash not found; using current ROS2 environment."
  echo "[nav2-basic-demo] Build first if packages are not discoverable:"
  echo "  colcon build --symlink-install"
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "FAIL: ros2 command not found. Source ROS2 first, for example:" >&2
  echo "  source /opt/ros/jazzy/setup.bash" >&2
  exit 1
fi

echo
echo "[nav2-basic-demo] Suggested terminal 1: start mock hardware, sensors, TF and cmd_vel safety chain"
echo "  ros2 launch robot_bringup bringup.launch.py mode:=hardware backend:=mock"
echo
echo "[nav2-basic-demo] Suggested terminal 2: start Nav2 basic"
echo "  ros2 launch robot_navigation nav.launch.py params_file:=${PARAMS_PATH} map:=${MAP_PATH} use_sim_time:=false"
echo
echo "[nav2-basic-demo] Suggested terminal 3: start RViz debug view"
echo "  rviz2 -d ${RVIZ_PATH}"
echo
echo "[nav2-basic-demo] RViz config:"
echo "  ${RVIZ_FILE}"
echo
echo "[nav2-basic-demo] Expected runtime topics:"
echo "  /map /scan /odom /cmd_vel /tf /global_costmap/costmap /local_costmap/costmap"
echo
echo "[nav2-basic-demo] After Nav2 is running, check topics with:"
echo "  bash scripts/check_nav2_runtime_topics.sh"

if [[ "${1:-}" == "--run-nav2" ]]; then
  echo
  echo "[nav2-basic-demo] Running Nav2 basic in this terminal."
  exec ros2 launch robot_navigation nav.launch.py \
    "params_file:=${PARAMS_PATH}" \
    "map:=${MAP_PATH}" \
    "use_sim_time:=false"
fi
