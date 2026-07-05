#!/usr/bin/env bash
set -euo pipefail

if ! command -v ros2 >/dev/null 2>&1; then
  echo "FAIL: ros2 command not found; source ROS2 and workspace setup first" >&2
  exit 1
fi

topics=(
  /amcl_pose
  /initialpose
  /localization/health
  /tf
  /tf_static
  /map
  /odom
)

topic_list="$(ros2 topic list)"
failed=0

for topic in "${topics[@]}"; do
  if grep -qx "${topic}" <<<"${topic_list}"; then
    echo "PASS: ${topic}"
  else
    echo "FAIL: ${topic}" >&2
    failed=1
  fi
done

exit "${failed}"
