#!/usr/bin/env bash
set -euo pipefail

fail() {
  echo "FAIL: $*" >&2
  exit 1
}

pass() {
  echo "PASS: $*"
}

if ! command -v ros2 >/dev/null 2>&1; then
  fail "ros2 command not found; run this in an Ubuntu/WSL shell with ROS2 sourced"
fi

echo "Checking Phase 4B safety runtime topics. Start bringup/Nav2 first."

TOPICS=(
  "/cmd_vel"
  "/muxed_cmd_vel"
  "/manual_takeover/state"
  "/emergency_stop/state"
  "/safety_state"
  "/safety/state"
  "/safety/reason"
  "/localization/health"
  "/scan"
  "/odom"
  "/chassis/state"
  "/tf"
  "/tf_static"
)

topic_list="$(ros2 topic list)"

missing=0
for topic in "${TOPICS[@]}"; do
  if grep -qx "${topic}" <<<"${topic_list}"; then
    pass "topic present: ${topic}"
  else
    echo "FAIL: topic missing: ${topic}" >&2
    missing=1
  fi
done

if [[ "${missing}" -ne 0 ]]; then
  exit 1
fi

pass "all expected safety runtime topics are present"
