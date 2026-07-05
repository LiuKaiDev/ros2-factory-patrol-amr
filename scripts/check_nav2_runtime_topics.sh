#!/usr/bin/env bash
set -euo pipefail

if ! command -v ros2 >/dev/null 2>&1; then
  echo "FAIL: ros2 command not found. Source ROS2 and the workspace before running this check." >&2
  exit 1
fi

TOPICS=(
  "/scan"
  "/tf"
  "/tf_static"
  "/odom"
  "/map"
  "/plan"
  "/cmd_vel"
  "/local_costmap/costmap"
  "/global_costmap/costmap"
)

echo "[nav2-runtime-topics] Checking topics from current ROS graph..."
topic_list="$(ros2 topic list 2>/tmp/check_nav2_runtime_topics.err || true)"

if [[ -z "${topic_list}" ]]; then
  echo "FAIL: ros2 topic list returned no topics. Is the Nav2 basic demo running?" >&2
  if [[ -s /tmp/check_nav2_runtime_topics.err ]]; then
    cat /tmp/check_nav2_runtime_topics.err >&2
  fi
  exit 1
fi

failed=0
for topic in "${TOPICS[@]}"; do
  if grep -Fxq "${topic}" <<<"${topic_list}"; then
    echo "PASS: ${topic}"
  else
    echo "FAIL: ${topic} missing" >&2
    failed=1
  fi
done

if [[ "${failed}" -ne 0 ]]; then
  echo "FAIL: one or more Nav2 runtime topics are missing" >&2
  exit 1
fi

echo "PASS: all expected Nav2 runtime topics are present"
