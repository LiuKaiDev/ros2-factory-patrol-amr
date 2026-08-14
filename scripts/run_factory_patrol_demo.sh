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
RVIZ_FILE="src/robot_simulation/rviz/factory_patrol_showcase.rviz"
LAUNCH_CMD="ros2 launch robot_bringup factory_patrol_demo.launch.py gui:=true use_rviz:=true"
PHASE5_LAUNCH_CMD="ros2 launch robot_bringup factory_patrol_demo.launch.py gui:=true use_rviz:=true use_nav2:=true use_detector:=true geometry_input_mode:=detector use_visual_inspection:=true perception_max_inference_rate_hz:=0.5 perception_tracking_params:=\$(ros2 pkg prefix --share robot_perception)/config/tracking_phase5_validation.yaml visual_inspection_params:=\$(ros2 pkg prefix --share robot_tasks)/config/visual_inspection_phase5_validation.yaml"
PHASE6_LAUNCH_CMD="ros2 launch robot_bringup factory_patrol_demo.launch.py gui:=false use_rviz:=false use_nav2:=true use_mission_runner:=false use_detector:=true geometry_input_mode:=detector use_perception_safety:=true perception_max_inference_rate_hz:=2.0 perception_tracking_params:=\$(ros2 pkg prefix --share robot_perception)/config/tracking_phase6_validation.yaml"

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
  bash scripts/run_factory_patrol_demo.sh --phase5
  bash scripts/run_factory_patrol_demo.sh --phase6

Default mode prints the launch command and expected runtime topics.
--launch starts the demo in this terminal.
--phase5 starts the detector, visual inspection task, existing navigation adapter, and Nav2.
--phase6 starts the detector, perception safety policy, existing Safety Gate, and Nav2.
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
echo "  debug RViz:   src/robot_simulation/rviz/factory_patrol_debug.rviz"
echo
echo "[factory-patrol-demo] Launch command:"
echo "  ${LAUNCH_CMD}"
echo "[factory-patrol-demo] Phase 5 launch command:"
echo "  ${PHASE5_LAUNCH_CMD}"
echo "[factory-patrol-demo] Phase 6 launch command:"
echo "  ${PHASE6_LAUNCH_CMD}"
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
  /perception/events
  /perception/safety_event
  /safety/state
  /safety/reason
  /inspection/observation_pose
  /inspection/status
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
  "--phase5")
    echo
    echo "[factory-patrol-demo] Running: ${PHASE5_LAUNCH_CMD}"
    exec ros2 launch robot_bringup factory_patrol_demo.launch.py \
      gui:=true use_rviz:=true use_nav2:=true use_detector:=true \
      geometry_input_mode:=detector use_visual_inspection:=true \
      perception_max_inference_rate_hz:=0.5 \
      perception_tracking_params:="$(ros2 pkg prefix --share robot_perception)/config/tracking_phase5_validation.yaml" \
      visual_inspection_params:="$(ros2 pkg prefix --share robot_tasks)/config/visual_inspection_phase5_validation.yaml"
    ;;
  "--phase6")
    echo
    echo "[factory-patrol-demo] Running: ${PHASE6_LAUNCH_CMD}"
    exec ros2 launch robot_bringup factory_patrol_demo.launch.py \
      gui:=false use_rviz:=false use_nav2:=true use_mission_runner:=false \
      use_detector:=true \
      geometry_input_mode:=detector use_perception_safety:=true \
      perception_max_inference_rate_hz:=2.0 \
      perception_tracking_params:="$(ros2 pkg prefix --share robot_perception)/config/tracking_phase6_validation.yaml"
    ;;
  *)
    echo "FAIL: unknown argument: ${1}" >&2
    usage >&2
    exit 1
    ;;
esac
