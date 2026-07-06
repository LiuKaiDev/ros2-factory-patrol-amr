#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

LAUNCH_FILE="src/robot_bringup/launch/factory_patrol_demo.launch.py"
STATIONS_FILE="src/robot_simulation/config/factory_patrol_stations.yaml"
ROUTE_FILE="src/robot_simulation/config/factory_patrol_route.yaml"
MISSION_FILE="src/robot_simulation/config/factory_patrol_multipoint_mission.yaml"
GOAL_PRINTER="scripts/print_factory_patrol_goals.py"
RVIZ_FILE="src/robot_simulation/rviz/factory_patrol_debug.rviz"
LAUNCH_CMD="ros2 launch robot_bringup factory_patrol_demo.launch.py use_rviz:=true use_mission_runner:=true mission_file:=${REPO_ROOT}/${MISSION_FILE}"

safe_source_setup() {
  local setup_file="$1"
  echo "[factory-multipoint] Sourcing ${setup_file}"
  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
}

usage() {
  cat <<EOF
Factory Patrol multipoint workflow helper

Usage:
  bash scripts/run_factory_patrol_multipoint_demo.sh
  bash scripts/run_factory_patrol_multipoint_demo.sh --launch

Default mode prints route goals, launch command, and follow-up commands.
--launch starts the demo in this terminal.
EOF
}

for file in "${LAUNCH_FILE}" "${STATIONS_FILE}" "${ROUTE_FILE}" "${MISSION_FILE}" "${GOAL_PRINTER}" "${RVIZ_FILE}"; do
  [[ -f "${file}" ]] || { echo "FAIL: missing ${file}" >&2; exit 1; }
done

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

echo "[factory-multipoint] Repository: ${REPO_ROOT}"
echo "[factory-multipoint] Route: start -> station_A -> station_B -> station_C -> dock"
echo "[factory-multipoint] RViz config: ${RVIZ_FILE}"
echo
python3 "${GOAL_PRINTER}"

if [[ -f "install/setup.bash" ]]; then
  safe_source_setup "install/setup.bash"
else
  echo "[factory-multipoint] install/setup.bash not found; using current ROS2 environment."
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "FAIL: ros2 command not found. Source ROS2 before running the demo." >&2
  echo "  source /opt/ros/jazzy/setup.bash" >&2
  exit 1
fi

echo
echo "[factory-multipoint] Launch command:"
echo "  ${LAUNCH_CMD}"
echo
echo "[factory-multipoint] Expected runtime topics:"
cat <<'EOF'
  /mission_runner/state
  /amr_simulation/markers
  /amr_simulation/demo_timeline
  /odom
  /scan
  /cmd_vel
EOF
echo
echo "[factory-multipoint] Start the mission profile manually after launch:"
echo "  ros2 service call /start_mission_profile std_srvs/srv/Trigger {}"
echo
echo "[factory-multipoint] Route YAML is a config asset; mission_runner consumes ${MISSION_FILE}."

case "${1:-}" in
  "")
    ;;
  "--launch"|"--run")
    echo
    echo "[factory-multipoint] Running: ${LAUNCH_CMD}"
    exec ros2 launch robot_bringup factory_patrol_demo.launch.py \
      use_rviz:=true \
      use_mission_runner:=true \
      "mission_file:=${REPO_ROOT}/${MISSION_FILE}"
    ;;
  *)
    echo "FAIL: unknown argument: ${1}" >&2
    usage >&2
    exit 1
    ;;
esac
