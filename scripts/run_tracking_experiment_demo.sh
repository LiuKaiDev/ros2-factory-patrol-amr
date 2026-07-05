#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${REPO_ROOT}"

CONTROLLER="${1:-pure_pursuit}"
if [[ "${CONTROLLER}" != "pure_pursuit" && "${CONTROLLER}" != "stanley" ]]; then
  echo "FAIL: controller must be pure_pursuit or stanley" >&2
  exit 1
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "FAIL: ros2 command not found. Source ROS2 first, for example:"
  echo "  source /opt/ros/jazzy/setup.bash"
  exit 1
fi

if [[ -f "install/setup.bash" ]]; then
  echo "[tracking-demo] Sourcing install/setup.bash"
  # shellcheck disable=SC1091
  source install/setup.bash
else
  echo "[tracking-demo] install/setup.bash not found; build first if packages are unavailable:"
  echo "  colcon build --symlink-install --packages-select robot_path_tracking robot_bringup"
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
CSV_PATH="${REPO_ROOT}/src/robot_experiments/results/tracking_${CONTROLLER}_${STAMP}.csv"

echo
echo "[tracking-demo] Suggested command:"
echo "  ros2 launch robot_bringup tracking.launch.py \\"
echo "    controller:=${CONTROLLER} \\"
echo "    use_mock_chassis:=true \\"
echo "    enable_csv_logging:=true \\"
echo "    csv_output_path:=${CSV_PATH} \\"
echo "    path_name:=demo_path"
echo
echo "[tracking-demo] After the run, analyze the CSV with:"
echo "  python3 scripts/analyze_tracking_result.py ${CSV_PATH}"
echo
echo "[tracking-demo] This script prints commands only; it does not modify files or start the demo."
