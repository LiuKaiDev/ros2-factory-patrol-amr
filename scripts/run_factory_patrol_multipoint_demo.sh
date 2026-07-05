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

for file in "${LAUNCH_FILE}" "${STATIONS_FILE}" "${ROUTE_FILE}" "${MISSION_FILE}" "${GOAL_PRINTER}"; do
  [[ -f "${file}" ]] || { echo "FAIL: missing ${file}" >&2; exit 1; }
done

echo "[factory-multipoint] Repository: ${REPO_ROOT}"
echo "[factory-multipoint] Route: start -> station_A -> station_B -> station_C -> dock"
echo
python3 "${GOAL_PRINTER}"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "FAIL: ros2 command not found. Source ROS2 before running the demo."
  echo "  source /opt/ros/jazzy/setup.bash"
  exit 1
fi

if [[ -f "install/setup.bash" ]]; then
  echo "[factory-multipoint] Sourcing install/setup.bash"
  # shellcheck disable=SC1091
  source install/setup.bash
else
  echo "[factory-multipoint] install/setup.bash not found; using current ROS2 environment."
fi

echo
echo "[factory-multipoint] Suggested launch:"
echo "  ros2 launch robot_bringup factory_patrol_demo.launch.py use_rviz:=true use_mission_runner:=true mission_file:=${REPO_ROOT}/${MISSION_FILE}"
echo
echo "[factory-multipoint] Start the mission profile manually after launch:"
echo "  ros2 service call /start_mission_profile std_srvs/srv/Trigger {}"
echo
echo "[factory-multipoint] Route YAML is a config asset; mission_runner consumes ${MISSION_FILE}."

if [[ "${1:-}" == "--run" ]]; then
  exec ros2 launch robot_bringup factory_patrol_demo.launch.py \
    use_rviz:=true \
    use_mission_runner:=true \
    "mission_file:=${REPO_ROOT}/${MISSION_FILE}"
fi
