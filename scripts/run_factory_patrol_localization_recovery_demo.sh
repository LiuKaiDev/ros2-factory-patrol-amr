#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

LAUNCH_FILE="src/robot_bringup/launch/factory_patrol_demo.launch.py"
CONFIG_FILE="src/robot_simulation/config/factory_patrol_localization_recovery.yaml"

[[ -f "${LAUNCH_FILE}" ]] || { echo "FAIL: missing ${LAUNCH_FILE}" >&2; exit 1; }
[[ -f "${CONFIG_FILE}" ]] || { echo "FAIL: missing ${CONFIG_FILE}" >&2; exit 1; }

BAD_X=3.80
BAD_Y=3.55
BAD_YAW=3.14
BAD_QZ=1.0
BAD_QW=0.000796
RECOVERY_X=-3.85
RECOVERY_Y=-3.65
RECOVERY_YAW=1.57
RECOVERY_QZ=0.706825
RECOVERY_QW=0.707388

echo "[factory-localization] Repository: ${REPO_ROOT}"
echo "[factory-localization] Config: ${CONFIG_FILE}"
echo
echo "[factory-localization] Start demo with localization health:"
echo "  ros2 launch robot_bringup factory_patrol_demo.launch.py use_rviz:=true use_localization_health:=true use_nav2:=true nav2_map:=/absolute/path/to/factory_patrol.yaml"
echo
echo "[factory-localization] Publish bad /initialpose:"
echo "  ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{header: {frame_id: \"map\"}, pose: {pose: {position: {x: ${BAD_X}, y: ${BAD_Y}, z: 0.0}, orientation: {z: ${BAD_QZ}, w: ${BAD_QW}}}, covariance: [0.25, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.25]}}'"
echo
echo "[factory-localization] Publish recovery /initialpose:"
echo "  ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{header: {frame_id: \"map\"}, pose: {pose: {position: {x: ${RECOVERY_X}, y: ${RECOVERY_Y}, z: 0.0}, orientation: {z: ${RECOVERY_QZ}, w: ${RECOVERY_QW}}}, covariance: [0.05, 0, 0, 0, 0, 0, 0, 0.05, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.05]}}'"
echo
echo "[factory-localization] Observe:"
echo "  ros2 topic echo /localization/health"
echo "  ros2 topic echo /safety/state"
echo "  ros2 topic echo /safety/reason"
echo "  ros2 topic echo /amcl_pose"
echo "  ros2 topic echo /tf"
echo
echo "[factory-localization] Expected state labels are listed in ${CONFIG_FILE}; no runtime transition is claimed here."

if ! command -v ros2 >/dev/null 2>&1; then
  echo "FAIL: ros2 command not found. Source ROS2 before injection." >&2
  exit 1
fi

if [[ -f "install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source install/setup.bash
fi

publish_pose() {
  local x="$1"
  local y="$2"
  local qz="$3"
  local qw="$4"
  local covariance="$5"
  ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
    "{header: {frame_id: \"map\"}, pose: {pose: {position: {x: ${x}, y: ${y}, z: 0.0}, orientation: {z: ${qz}, w: ${qw}}}, covariance: ${covariance}}}"
}

if [[ "${1:-}" == "--inject-bad-pose" ]]; then
  publish_pose "${BAD_X}" "${BAD_Y}" "${BAD_QZ}" "${BAD_QW}" "[0.25, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.25]"
elif [[ "${1:-}" == "--inject-recovery-pose" ]]; then
  publish_pose "${RECOVERY_X}" "${RECOVERY_Y}" "${RECOVERY_QZ}" "${RECOVERY_QW}" "[0.05, 0, 0, 0, 0, 0, 0, 0.05, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.05]"
fi
