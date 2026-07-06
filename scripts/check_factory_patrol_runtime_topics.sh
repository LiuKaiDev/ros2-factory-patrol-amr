#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

safe_source_setup() {
  local setup_file="$1"
  echo "[factory-topic-check] Sourcing ${setup_file}"
  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
}

if [[ -f "install/setup.bash" ]]; then
  safe_source_setup "install/setup.bash"
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "FAIL: ros2 command not found. Source ROS2 first, for example:" >&2
  echo "  source /opt/ros/jazzy/setup.bash" >&2
  exit 1
fi

TOPICS=(
  "/clock"
  "/tf"
  "/joint_states"
  "/odom"
  "/scan"
  "/cmd_vel"
  "/mission_runner/state"
  "/safety_state"
  "/localization/health"
  "/amr_simulation/markers"
  "/amr_simulation/demo_timeline"
)

WARN_ONLY_TOPICS=(
  "/amr_simulation/demo_timeline"
)

is_warn_only_topic() {
  local topic="$1"
  local item
  for item in "${WARN_ONLY_TOPICS[@]}"; do
    [[ "${item}" == "${topic}" ]] && return 0
  done
  return 1
}

publisher_count() {
  local topic_info="$1"
  awk -F': ' '/Publisher count:/ {print $2; exit}' <<<"${topic_info}"
}

subscription_count() {
  local topic_info="$1"
  awk -F': ' '/Subscription count:/ {print $2; exit}' <<<"${topic_info}"
}

print_topic_info() {
  local topic="$1"
  local info
  if info="$(timeout 5s ros2 topic info "${topic}" -v 2>/dev/null)"; then
    local publishers subscribers
    publishers="$(publisher_count "${info}")"
    subscribers="$(subscription_count "${info}")"
    echo "      publishers=${publishers:-unknown} subscribers=${subscribers:-unknown}"
    if [[ "${topic}" == "/scan" ]]; then
      if grep -qi "RELIABLE" <<<"${info}"; then
        echo "      /scan QoS hint: RELIABLE publisher detected; RViz should use Reliable."
      elif grep -qi "BEST_EFFORT" <<<"${info}"; then
        echo "      /scan QoS hint: BEST_EFFORT publisher detected; RViz should use Best Effort."
      else
        echo "      /scan QoS hint: unable to infer reliability from ros2 topic info."
      fi
    fi
    if [[ "${topic}" == "/amr_simulation/demo_timeline" && "${publishers:-0}" == "0" ]]; then
      echo "      WARN: demo_timeline currently has no publisher; this can be normal before a demo director publishes."
    fi
    return 0
  fi
  echo "      WARN: unable to read ros2 topic info for ${topic}"
  return 1
}

echo_once_field() {
  local topic="$1"
  local field="$2"
  timeout 6s ros2 topic echo --once "${topic}" --field "${field}" 2>/dev/null | sed '/^$/d' | head -n 1
}

first_marker_frame() {
  timeout 6s ros2 topic echo --once /amr_simulation/markers 2>/dev/null |
    awk -F': ' '/frame_id:/ {gsub("\"", "", $2); print $2; exit}'
}

check_tf() {
  local target="$1"
  if timeout 3s ros2 run tf2_ros tf2_echo odom "${target}" >/tmp/factory_patrol_tf_check.log 2>&1; then
    echo "PASS: TF odom -> ${target}"
    return 0
  fi
  if grep -qi "At time" /tmp/factory_patrol_tf_check.log; then
    echo "PASS: TF odom -> ${target}"
    return 0
  fi
  echo "WARN: TF odom -> ${target} not observed within timeout"
  return 1
}

echo "[factory-topic-check] Checking Factory Patrol runtime topics..."
if ! topic_list="$(timeout 8s ros2 topic list 2>/dev/null)"; then
  echo "FAIL: unable to read ROS2 topic list within timeout" >&2
  exit 1
fi

failures=0
for topic in "${TOPICS[@]}"; do
  if grep -Fxq "${topic}" <<<"${topic_list}"; then
    echo "PASS: ${topic}"
    print_topic_info "${topic}" || true
  else
    if is_warn_only_topic "${topic}"; then
      echo "WARN: missing ${topic} (optional timeline topic before demo publisher starts)"
    else
      echo "FAIL: missing ${topic}" >&2
      failures=$((failures + 1))
    fi
  fi
done

echo
echo "[factory-topic-check] Sampling message frames..."
scan_frame="$(echo_once_field "/scan" "header.frame_id" || true)"
if [[ -n "${scan_frame}" ]]; then
  echo "PASS: /scan frame_id=${scan_frame}"
else
  echo "WARN: unable to sample /scan header.frame_id within timeout"
fi

marker_frame="$(first_marker_frame || true)"
if [[ -n "${marker_frame}" ]]; then
  echo "PASS: /amr_simulation/markers first marker frame_id=${marker_frame}"
else
  echo "WARN: unable to sample /amr_simulation/markers marker frame_id within timeout"
fi

echo
echo "[factory-topic-check] Checking odom TF connectivity..."
if ! check_tf "base_link"; then
  check_tf "base_footprint" || true
fi

if [[ "${failures}" -eq 0 ]]; then
  echo "[factory-topic-check] PASS: all expected Factory Patrol topics are present."
  exit 0
fi

echo "[factory-topic-check] FAIL: ${failures} expected topic(s) missing." >&2
exit 1
