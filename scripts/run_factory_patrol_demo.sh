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
RVIZ_FILE="src/robot_simulation/rviz/nav2_basic_debug.rviz"

echo "[factory-patrol-demo] Repository: ${REPO_ROOT}"

for file in "${LAUNCH_FILE}" "${WORLD_FILE}" "${STATIONS_FILE}" "${ZONES_FILE}" "${ROUTE_FILE}" "${RVIZ_FILE}"; do
  if [[ ! -f "${file}" ]]; then
    echo "FAIL: missing ${file}" >&2
    exit 1
  fi
done

if ! command -v ros2 >/dev/null 2>&1; then
  echo "FAIL: ros2 command not found. Source ROS2 first, for example:"
  echo "  source /opt/ros/jazzy/setup.bash"
  exit 1
fi

if [[ -f "install/setup.bash" ]]; then
  echo "[factory-patrol-demo] Sourcing install/setup.bash"
  # shellcheck disable=SC1091
  source install/setup.bash
else
  echo "[factory-patrol-demo] install/setup.bash not found; using current ROS2 environment."
  echo "[factory-patrol-demo] Build first if packages are not discoverable:"
  echo "  colcon build --symlink-install"
fi

echo
echo "[factory-patrol-demo] Factory patrol assets:"
echo "  world:    ${WORLD_FILE}"
echo "  stations: ${STATIONS_FILE}"
echo "  zones:    ${ZONES_FILE}"
echo "  route:    ${ROUTE_FILE}"
echo
echo "[factory-patrol-demo] Suggested launch:"
echo "  ros2 launch robot_bringup factory_patrol_demo.launch.py gui:=true use_rviz:=true"
echo
echo "[factory-patrol-demo] Nav2 is disabled by default because a measured factory_patrol map"
echo "[factory-patrol-demo] is not committed in Phase 5A. To experiment with an explicit map:"
echo "  ros2 launch robot_bringup factory_patrol_demo.launch.py use_nav2:=true nav2_map:=/absolute/path/to/factory_patrol.yaml"

if [[ "${1:-}" == "--run" ]]; then
  echo
  echo "[factory-patrol-demo] Running factory patrol demo launch."
  exec ros2 launch robot_bringup factory_patrol_demo.launch.py gui:=true use_rviz:=true
fi
