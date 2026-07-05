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

HELPER_HPP="src/robot_teleop/include/robot_teleop/cmd_vel_safety.hpp"
GATE_CPP="src/robot_teleop/src/cmd_vel_safety_gate_node.cpp"
TEST_CPP="src/robot_teleop/test/cmd_vel_safety_test.cpp"
LAUNCH_FILE="src/robot_teleop/launch/cmd_vel_stack.launch.py"
CMAKE_FILE="src/robot_teleop/CMakeLists.txt"
PACKAGE_FILE="src/robot_teleop/package.xml"
SAFETY_DOC="docs/safety_state_machine.md"
LOCALIZATION_DOC="docs/localization.md"
CHASSIS_DOC="docs/chassis_protocol.md"
ARCH_DOC="docs/architecture.md"
README_FILE="README.md"

for file in "${HELPER_HPP}" "${GATE_CPP}" "${TEST_CPP}" "${LAUNCH_FILE}" \
  "${CMAKE_FILE}" "${PACKAGE_FILE}" "${SAFETY_DOC}" "${LOCALIZATION_DOC}" \
  "${CHASSIS_DOC}" "${ARCH_DOC}" "${README_FILE}"; do
  require_file "${file}"
done

for state in NORMAL MANUAL_TAKEOVER SPEED_LIMITED SENSOR_STALE LOCALIZATION_LOST \
  CHASSIS_FAULT COMMUNICATION_LOST EMERGENCY_STOP RECOVERY; do
  require_grep "${state}" "${HELPER_HPP}" "safety helper does not define ${state}"
  require_grep "${state}" "${SAFETY_DOC}" "safety docs do not describe ${state}"
done

require_grep "ResolveHighestPriority" "${HELPER_HPP}" \
  "safety helper is missing priority resolution"
require_grep "PolicyForState" "${HELPER_HPP}" \
  "safety helper is missing output policy"
require_grep "SafetyStateFromChassisStatus" "${HELPER_HPP}" \
  "safety helper is missing chassis fault mapping"
require_grep "CMD_TIMEOUT" "${HELPER_HPP}" \
  "CMD_TIMEOUT mapping is missing"
require_grep "HEARTBEAT_TIMEOUT" "${HELPER_HPP}" \
  "HEARTBEAT_TIMEOUT mapping is missing"
require_grep "BACKEND_DISCONNECTED" "${HELPER_HPP}" \
  "BACKEND_DISCONNECTED mapping is missing"
require_grep "MALFORMED_PACKET" "${HELPER_HPP}" \
  "MALFORMED_PACKET mapping is missing"
require_grep "ESTOP_ACTIVE" "${HELPER_HPP}" \
  "ESTOP_ACTIVE mapping is missing"
require_grep "SafetyStateFromLocalizationHealth" "${HELPER_HPP}" \
  "safety helper is missing localization health mapping"

require_grep "/localization/health" "${GATE_CPP}" \
  "safety gate does not subscribe to default localization health topic"
require_grep "/scan" "${GATE_CPP}" \
  "safety gate does not monitor /scan"
require_grep "/odom" "${GATE_CPP}" \
  "safety gate does not monitor /odom"
require_grep "/chassis/state" "${GATE_CPP}" \
  "safety gate does not monitor /chassis/state"
require_grep "/safety/state" "${GATE_CPP}" \
  "safety gate does not publish /safety/state"
require_grep "/safety/reason" "${GATE_CPP}" \
  "safety gate does not publish /safety/reason"

require_grep "localization_health_topic" "${LAUNCH_FILE}" \
  "launch file does not expose localization health topic"
require_grep "scan_timeout_sec" "${LAUNCH_FILE}" \
  "launch file does not expose scan timeout"
require_grep "speed_limited_max_linear_mps" "${LAUNCH_FILE}" \
  "launch file does not expose speed limited policy"
require_grep "nav_msgs" "${CMAKE_FILE}" \
  "robot_teleop CMake missing nav_msgs dependency"
require_grep "nav_msgs" "${PACKAGE_FILE}" \
  "robot_teleop package.xml missing nav_msgs dependency"
require_grep "SafetyStateFromChassisStatus" "${TEST_CPP}" \
  "safety state helper tests are missing chassis mapping coverage"
require_grep "ApplySafetyPolicy_SpeedLimitedClampsCommand" "${TEST_CPP}" \
  "safety state helper tests are missing speed-limited coverage"

require_grep "/safety/state" "${SAFETY_DOC}" \
  "safety docs do not document /safety/state"
require_grep "Phase 4B" "${LOCALIZATION_DOC}" \
  "localization docs do not mention Phase 4B safety integration"
require_grep "CMD_TIMEOUT" "${CHASSIS_DOC}" \
  "chassis docs do not document CMD_TIMEOUT safety mapping"
require_grep "cmd_vel_safety_gate" "${ARCH_DOC}" \
  "architecture docs do not mention cmd_vel_safety_gate integration"
require_grep "check_safety_state_machine.sh" "${README_FILE}" \
  "README does not list safety check script"

pass "safety state helper, cmd_vel safety gate integration, launch parameters, tests, scripts, and docs are present"
