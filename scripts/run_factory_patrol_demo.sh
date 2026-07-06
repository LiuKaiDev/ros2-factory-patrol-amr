#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

LAUNCH_FILE="src/robot_bringup/launch/factory_patrol_demo.launch.py"
WORLD_FILE="src/robot_simulation/worlds/factory_patrol.sdf"
STATIONS_FILE="src/robot_simulation/config/factory_patrol_stations.yaml"
ZONES_FILE="src/robot_simulation/config/factory_patrol_zones.yaml"
ROUTE_FILE="src/robot_simulation/config/factory_patrol_route.yaml"
RVIZ_FILE="src/robot_simulation/rviz/factory_patrol_debug.rviz"
LAUNCH_CMD="ros2 launch robot_bringup factory_patrol_demo.launch.py gui:=true use_rviz:=true"

safe_source_setup() {
  local setup_file="$1"
  echo "[factory-patrol-demo] Sourcing ${setup_file}"
  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
}

usage() {
  cat <<EOF
Factory Patrol Gazebo + RViz demo helper

Usage:
  bash scripts/run_factory_patrol_demo.sh
  bash scripts/run_factory_patrol_demo.sh --launch

Default mode prints the launch command and expected runtime topics.
--launch starts the demo in this terminal.
EOF
}

for file in "${LAUNCH_FILE}" "${WORLD_FILE}" "${STATIONS_FILE}" "${ZONES_FILE}" "${ROUTE_FILE}" "${RVIZ_FILE}"; do
  if [[ ! -f "${file}" ]]; then
    echo "FAIL: missing ${file}" >&2
    exit 1
  fi
done

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if [[ -f "install/setup.bash" ]]; then
  safe_source_setup "install/setup.bash"
else
  echo "[factory-patrol-demo] install/setup.bash not found; using current ROS2 environment."
  echo "[factory-patrol-demo] Build first if packages are not discoverable:"
  echo "  colcon build --symlink-install"
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "FAIL: ros2 command not found. Source ROS2 first, for example:" >&2
  echo "  source /opt/ros/jazzy/setup.bash" >&2
  exit 1
fi

echo "[factory-patrol-demo] Repository: ${REPO_ROOT}"
echo
echo "[factory-patrol-demo] Factory Patrol assets:"
echo "  world:       ${WORLD_FILE}"
echo "  stations:    ${STATIONS_FILE}"
echo "  zones:       ${ZONES_FILE}"
echo "  route:       ${ROUTE_FILE}"
echo "  RViz config: ${RVIZ_FILE}"
echo
echo "[factory-patrol-demo] Launch command:"
echo "  ${LAUNCH_CMD}"
echo
echo "[factory-patrol-demo] Expected runtime topics:"
cat <<'EOF'
  /clock
  /tf
  /joint_states
  /odom
  /scan
  /cmd_vel
  /mission_runner/state
  /safety_state
  /localization/health
  /amr_simulation/markers
  /amr_simulation/demo_timeline
EOF
echo
echo "[factory-patrol-demo] Runtime check after launch:"
echo "  bash scripts/check_factory_patrol_runtime_topics.sh"

case "${1:-}" in
  "")
    ;;
  "--launch"|"--run")
    echo
    echo "[factory-patrol-demo] Running: ${LAUNCH_CMD}"
    exec ros2 launch robot_bringup factory_patrol_demo.launch.py gui:=true use_rviz:=true
    ;;
  *)
    echo "FAIL: unknown argument: ${1}" >&2
    usage >&2
    exit 1
    ;;
esac
