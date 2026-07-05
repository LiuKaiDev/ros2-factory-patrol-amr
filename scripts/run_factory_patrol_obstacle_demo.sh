#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

LAUNCH_FILE="src/robot_bringup/launch/factory_patrol_demo.launch.py"
OBSTACLE_CONFIG="src/robot_simulation/config/factory_patrol_obstacle_demo.yaml"
MODEL_SDF="src/robot_simulation/models/temporary_box_obstacle/model.sdf"

for file in "${LAUNCH_FILE}" "${OBSTACLE_CONFIG}" "${MODEL_SDF}"; do
  [[ -f "${file}" ]] || { echo "FAIL: missing ${file}" >&2; exit 1; }
done

echo "[factory-obstacle] Repository: ${REPO_ROOT}"
echo "[factory-obstacle] Obstacle config: ${OBSTACLE_CONFIG}"
echo "[factory-obstacle] Suggested segment: station_A_to_station_B"
echo
echo "[factory-obstacle] Start the factory patrol demo first:"
echo "  ros2 launch robot_bringup factory_patrol_demo.launch.py use_rviz:=true use_nav2:=true nav2_map:=/absolute/path/to/factory_patrol.yaml"
echo
echo "[factory-obstacle] Suggested temporary obstacle spawn command for ros_gz_sim:"
echo "  ros2 run ros_gz_sim create -world factory_patrol -name temporary_box_obstacle -file ${REPO_ROOT}/${MODEL_SDF} -x -2.35 -y 1.85 -z 0.35 -Y 0.0"
echo
echo "[factory-obstacle] Observe:"
echo "  ros2 topic echo /scan --once"
echo "  ros2 topic echo /local_costmap/costmap --once"
echo "  ros2 topic echo /safety/state --once"
echo "  ros2 topic echo /cmd_vel --once"
echo
echo "[factory-obstacle] No avoidance result is claimed until these topics are captured in a real run."

if ! command -v ros2 >/dev/null 2>&1; then
  echo "FAIL: ros2 command not found. This script is a command guide unless ROS2 is sourced." >&2
  exit 1
fi

if [[ -f "install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source install/setup.bash
fi

if [[ "${1:-}" == "--spawn" ]]; then
  exec ros2 run ros_gz_sim create \
    -world factory_patrol \
    -name temporary_box_obstacle \
    -file "${REPO_ROOT}/${MODEL_SDF}" \
    -x -2.35 -y 1.85 -z 0.35 -Y 0.0
fi
