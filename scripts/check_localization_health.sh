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

NODE_CPP="src/robot_navigation/src/localization_health_monitor_node.cpp"
HELPER_HPP="src/robot_navigation/include/robot_navigation/localization_health.hpp"
HELPER_CPP="src/robot_navigation/src/localization_health.cpp"
TEST_CPP="src/robot_navigation/test/localization_health_test.cpp"
LAUNCH_FILE="src/robot_navigation/launch/localization_health.launch.py"
CMAKE_FILE="src/robot_navigation/CMakeLists.txt"
LOCALIZATION_DOC="docs/localization.md"
SAFETY_DOC="docs/safety_state_machine.md"

for file in "${NODE_CPP}" "${HELPER_HPP}" "${HELPER_CPP}" "${TEST_CPP}" \
  "${LAUNCH_FILE}" "${CMAKE_FILE}" "${LOCALIZATION_DOC}" "${SAFETY_DOC}"; do
  require_file "${file}"
done

require_grep "localization_health_monitor_node" "${NODE_CPP}" \
  "localization_health_monitor_node implementation is missing"
require_grep "LOCALIZATION_OK" "${HELPER_CPP}" \
  "LOCALIZATION_OK state is missing"
require_grep "LOCALIZATION_UNSTABLE" "${HELPER_CPP}" \
  "LOCALIZATION_UNSTABLE state is missing"
require_grep "LOCALIZATION_LOST" "${HELPER_CPP}" \
  "LOCALIZATION_LOST state is missing"
require_grep "LOCALIZATION_RECOVERING" "${HELPER_CPP}" \
  "LOCALIZATION_RECOVERING state is missing"
require_grep "LOCALIZATION_RECOVERED" "${HELPER_CPP}" \
  "LOCALIZATION_RECOVERED state is missing"
require_grep "amcl_pose_topic" "${NODE_CPP}" \
  "AMCL pose subscription parameter is missing"
require_grep "/amcl_pose" "${NODE_CPP}" \
  "default /amcl_pose topic is missing"
require_grep "health_topic" "${NODE_CPP}" \
  "health topic parameter is missing"
require_grep "/localization/health" "${NODE_CPP}" \
  "default /localization/health topic is missing"
require_grep "covariance\\[0\\]" "${HELPER_CPP}" \
  "x covariance check is missing"
require_grep "covariance\\[7\\]" "${HELPER_CPP}" \
  "y covariance check is missing"
require_grep "covariance\\[35\\]" "${HELPER_CPP}" \
  "yaw covariance check is missing"
require_grep "canTransform" "${NODE_CPP}" \
  "TF availability check is missing"
require_grep "localization_health_monitor_node" "${LAUNCH_FILE}" \
  "localization health launch content is missing"
require_grep "localization_health_monitor_node" "${CMAKE_FILE}" \
  "localization health node is not registered in CMake"
require_grep "localization_health_test" "${CMAKE_FILE}" \
  "localization health test is not registered in CMake"
require_grep "/localization/health" "${LOCALIZATION_DOC}" \
  "docs/localization.md does not document /localization/health"
require_grep "LOCALIZATION_LOST" "${SAFETY_DOC}" \
  "docs/safety_state_machine.md does not document LOCALIZATION_LOST"

pass "localization health monitor, states, covariance checks, TF checks, launch, tests, and docs are present"
