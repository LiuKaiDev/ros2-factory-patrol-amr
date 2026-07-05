#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

fail() {
  echo "FAIL: $*" >&2
  exit 1
}

pass() {
  echo "PASS: $*"
}

require_file() {
  local file="$1"
  [[ -f "${ROOT_DIR}/${file}" ]] || fail "missing file: ${file}"
}

require_grep() {
  local pattern="$1"
  local file="$2"
  local message="$3"
  grep -q "${pattern}" "${ROOT_DIR}/${file}" || fail "${message}"
}

DRIVER_CPP="src/robot_hardware/src/chassis_driver_node.cpp"
ADAPTER_HPP="src/robot_hardware/include/robot_hardware/chassis_system_adapter.hpp"
KINEMATICS_CPP="src/robot_hardware/src/chassis_kinematics.cpp"
KINEMATICS_TEST="src/robot_hardware/test/chassis_kinematics_test.cpp"
CMAKE_FILE="src/robot_hardware/CMakeLists.txt"
CALIBRATION_DOC="docs/calibration.md"
CHASSIS_DOC="docs/chassis_protocol.md"

for file in "${DRIVER_CPP}" "${ADAPTER_HPP}" "${KINEMATICS_CPP}" "${KINEMATICS_TEST}" \
  "${CMAKE_FILE}" "${CALIBRATION_DOC}" "${CHASSIS_DOC}"; do
  require_file "${file}"
done

require_grep "wheel_diameter_m" "${DRIVER_CPP}" \
  "wheel_diameter_m parameter is missing from chassis_driver_node"
require_grep "track_width_m" "${DRIVER_CPP}" \
  "track_width_m parameter is missing from chassis_driver_node"
require_grep "max_linear_speed_mps" "${DRIVER_CPP}" \
  "max_linear_speed_mps parameter is missing from chassis_driver_node"
require_grep "max_angular_speed_radps" "${DRIVER_CPP}" \
  "max_angular_speed_radps parameter is missing from chassis_driver_node"
require_grep "odom_pose_covariance_x" "${DRIVER_CPP}" \
  "odom pose covariance parameters are missing"
require_grep "odom_twist_covariance_vx" "${DRIVER_CPP}" \
  "odom twist covariance parameters are missing"
require_grep "pose.covariance" "${DRIVER_CPP}" \
  "pose covariance is not filled in odometry publishing"
require_grep "twist.covariance" "${DRIVER_CPP}" \
  "twist covariance is not filled in odometry publishing"
require_grep "track_width_m = 0.43" "${ADAPTER_HPP}" \
  "adapter default track_width_m is not aligned to 0.43"

require_grep "EstimateWheelSpeeds" "${KINEMATICS_TEST}" \
  "chassis kinematics wheel speed tests are missing"
require_grep "EstimateCommandFromWheelSpeeds" "${KINEMATICS_TEST}" \
  "chassis kinematics inverse tests are missing"
require_grep "chassis_kinematics_test" "${CMAKE_FILE}" \
  "chassis_kinematics_test is not registered in CMake"

require_grep "wheel_diameter_m" "${CALIBRATION_DOC}" \
  "calibration doc does not describe wheel_diameter_m"
require_grep "track_width_m" "${CALIBRATION_DOC}" \
  "calibration doc does not describe track_width_m"
require_grep "odom_pose_covariance" "${CALIBRATION_DOC}" \
  "calibration doc does not describe odom pose covariance"
require_grep "TBD" "${CALIBRATION_DOC}" \
  "calibration doc should keep real measurement values as TBD"
require_grep "Phase 3B" "${CHASSIS_DOC}" \
  "chassis protocol doc does not include Phase 3B notes"

pass "chassis odom covariance, calibration parameters, docs, and kinematics tests are present"
