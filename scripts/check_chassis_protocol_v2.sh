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

PACKET_HPP="src/robot_hardware/include/robot_hardware/chassis_packet.hpp"
PACKET_CPP="src/robot_hardware/src/chassis_packet.cpp"
ADAPTER_HPP="src/robot_hardware/include/robot_hardware/chassis_system_adapter.hpp"
ADAPTER_CPP="src/robot_hardware/src/chassis_system_adapter.cpp"
DRIVER_CPP="src/robot_hardware/src/chassis_driver_node.cpp"
SIMULATOR_CPP="src/robot_hardware/src/chassis_simulator_node.cpp"
PACKET_TEST="src/robot_hardware/test/chassis_packet_test.cpp"
ADAPTER_TEST="src/robot_hardware/test/chassis_system_adapter_test.cpp"

for file in "${PACKET_HPP}" "${PACKET_CPP}" "${ADAPTER_HPP}" "${ADAPTER_CPP}" \
  "${DRIVER_CPP}" "${SIMULATOR_CPP}" "${PACKET_TEST}" "${ADAPTER_TEST}"; do
  require_file "${file}"
done

require_grep "enum class ChassisFaultCode" "${PACKET_HPP}" \
  "ChassisFaultCode enum is missing"
require_grep "struct ChassisCommandFrameV2" "${PACKET_HPP}" \
  "ChassisCommandFrameV2 is missing"
require_grep "struct ChassisHeartbeatPacket" "${PACKET_HPP}" \
  "ChassisHeartbeatPacket is missing"
require_grep "EncodeCommandV2" "${PACKET_CPP}" \
  "CMDV2 encoder is missing"
require_grep "ParseCommandFrameV2" "${PACKET_CPP}" \
  "CMDV2 parser is missing"
require_grep "STATEV2" "${PACKET_CPP}" \
  "STATEV2 parser/encoder is missing"
require_grep "HEARTBEATV2" "${PACKET_CPP}" \
  "HEARTBEATV2 parser/encoder is missing"

require_grep "WriteHeartbeat" "${ADAPTER_HPP}" \
  "adapter heartbeat write API is missing"
require_grep "last_rx_seq" "${ADAPTER_HPP}" \
  "adapter receive sequence state is missing"
require_grep "ChassisFaultCode::kMalformedPacket" "${ADAPTER_CPP}" \
  "adapter malformed packet fault mapping is missing"

require_grep "heartbeat_period_ms" "${DRIVER_CPP}" \
  "driver heartbeat period parameter is missing"
require_grep "heartbeat_timeout_ms" "${DRIVER_CPP}" \
  "driver heartbeat timeout parameter is missing"
require_grep "command_timeout_ms" "${DRIVER_CPP}" \
  "driver command timeout parameter is missing"
require_grep "EnforceCommandTimeout" "${DRIVER_CPP}" \
  "driver command timeout stop path is missing"
require_grep "SendHeartbeatIfDue" "${DRIVER_CPP}" \
  "driver heartbeat transmit path is missing"
require_grep "fault_code=" "${DRIVER_CPP}" \
  "driver chassis state fault code status is missing"
require_grep "ParseCommandFrameV2" "${SIMULATOR_CPP}" \
  "simulator CMDV2 receive compatibility is missing"
require_grep "EncodeStateV2" "${SIMULATOR_CPP}" \
  "simulator STATEV2 transmit compatibility is missing"

require_grep "EncodeCommandV2_ValidFrame" "${PACKET_TEST}" \
  "CMDV2 unit test is missing"
require_grep "ParsePacketLine_ValidStateV2" "${PACKET_TEST}" \
  "STATEV2 unit test is missing"
require_grep "ParsePacketLine_ValidHeartbeatV2" "${PACKET_TEST}" \
  "HEARTBEATV2 unit test is missing"
require_grep "Write_CommandFrameV2" "${ADAPTER_TEST}" \
  "adapter v2 write unit test is missing"
require_grep "Read_TextV2Packets" "${ADAPTER_TEST}" \
  "adapter v2 read unit test is missing"

pass "chassis protocol v2 heartbeat, fault code, timeout hooks, and tests are present"
