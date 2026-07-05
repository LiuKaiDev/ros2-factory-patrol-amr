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

echo "Checking factory patrol runtime topics. Start the demo/Nav2 first."
topic_list="$(ros2 topic list)"

TOPICS=(
  "/scan"
  "/tf"
  "/tf_static"
  "/odom"
  "/map"
  "/amcl_pose"
  "/localization/health"
  "/safety/state"
  "/safety/reason"
  "/cmd_vel"
  "/plan"
  "/local_costmap/costmap"
  "/global_costmap/costmap"
)

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

pass "all expected factory patrol runtime topics are present"
