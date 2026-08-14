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
  "/camera/color/image_raw"
  "/camera/depth/image_raw"
  "/camera/color/camera_info"
  "/perception/geometry/camera_point"
  "/perception/geometry/map_point"
  "/perception/markers"
  "/perception/objects_3d"
)

DETECTOR_MODE="${FACTORY_PATROL_DETECTOR_MODE:-false}"
VISUAL_INSPECTION_MODE="${FACTORY_PATROL_VISUAL_INSPECTION_MODE:-false}"
PERCEPTION_SAFETY_MODE="${FACTORY_PATROL_PERCEPTION_SAFETY_MODE:-false}"
PERCEPTION_DIAGNOSTICS_MODE="${FACTORY_PATROL_PERCEPTION_DIAGNOSTICS_MODE:-false}"
if [[ "${DETECTOR_MODE}" == "true" ]]; then
  TOPICS+=("/perception/detections_2d" "/perception/debug_image")
fi
if [[ "${PERCEPTION_SAFETY_MODE}" == "true" ]]; then
  TOPICS+=(
    "/perception/safety_event"
    "/safety/state"
    "/safety/reason"
    "/nav2_cmd_vel"
  )
fi
if [[ "${PERCEPTION_DIAGNOSTICS_MODE}" == "true" ]]; then
  TOPICS+=("/perception/diagnostics" "/system_health" "/fault_supervisor/state" "/diagnostics")
fi
if [[ "${PERCEPTION_SAFETY_MODE}" == "true" && "${VISUAL_INSPECTION_MODE}" != "true" ]]; then
  for index in "${!TOPICS[@]}"; do
    [[ "${TOPICS[$index]}" == "/mission_runner/state" ]] && unset 'TOPICS[index]'
  done
  echo "[factory-topic-check] Phase 6 safety profile omits /mission_runner/state by design."
fi
if [[ "${VISUAL_INSPECTION_MODE}" == "true" ]]; then
  TOPICS+=(
    "/perception/events"
    "/inspection/observation_pose"
    "/inspection/observation_marker"
    "/inspection/status"
    "/navigate_sequence/current_goal"
    "/nav2_cmd_vel"
  )
fi

check_managed_target() {
  if timeout 30s ros2 topic echo --once --timeout 28 /perception/objects_3d \
      --filter "m.header.frame_id == 'map' and (m.header.stamp.sec > 0 or m.header.stamp.nanosec > 0) and m.target_id > 0 and len(m.class_name) > 0 and 0.0 <= m.confidence <= 1.0 and m.depth_valid and m.tracking_state in (0, 1, 2, 3) and m.position.x == m.position.x and m.position.y == m.position.y and m.position.z == m.position.z" \
      --no-arr >/dev/null 2>&1; then
    echo "PASS: /perception/objects_3d publishes a finite managed map target"
    return 0
  fi
  echo "FAIL: /perception/objects_3d did not publish a valid managed target" >&2
  return 1
}

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

wait_for_topic() {
  local topic="$1"
  local attempt
  for attempt in $(seq 1 20); do
    if [[ -n "$(timeout 3s ros2 topic type "${topic}" 2>/dev/null || true)" ]]; then
      return 0
    fi
    sleep 0.25
  done
  return 1
}

echo_once_field() {
  local topic="$1"
  local field="$2"
  timeout 6s ros2 topic echo --once "${topic}" --field "${field}" 2>/dev/null | sed '/^$/d' | head -n 1
}

check_topic_type() {
  local topic="$1"
  local expected_type="$2"
  local actual_type
  actual_type="$(timeout 5s ros2 topic type "${topic}" 2>/dev/null || true)"
  if [[ "${actual_type}" == "${expected_type}" ]]; then
    echo "PASS: ${topic} type=${actual_type}"
    return 0
  fi
  echo "FAIL: ${topic} expected type ${expected_type}, got ${actual_type:-none}" >&2
  return 1
}

check_image_message() {
  local topic="$1"
  if timeout 10s ros2 topic echo --once --timeout 8 "${topic}" \
      --filter 'm.width > 0 and m.height > 0 and len(m.data) > 0' \
      --no-arr >/dev/null 2>&1; then
    echo "PASS: ${topic} has nonempty image data"
    return 0
  fi
  echo "FAIL: ${topic} did not produce nonempty image data" >&2
  return 1
}

check_camera_info() {
  if timeout 10s ros2 topic echo --once --timeout 8 /camera/color/camera_info \
      --filter 'm.width > 0 and m.height > 0 and len(m.k) == 9 and m.k[0] > 0.0 and m.k[4] > 0.0 and m.k[2] > 0.0 and m.k[5] > 0.0 and m.k[8] == 1.0' \
      --no-arr >/dev/null 2>&1; then
    echo "PASS: /camera/color/camera_info has valid nonzero intrinsics"
    return 0
  fi
  echo "FAIL: /camera/color/camera_info intrinsics are missing or invalid" >&2
  return 1
}

check_camera_frame() {
  local topic="$1"
  local frame
  frame="$(echo_once_field "${topic}" "header.frame_id" || true)"
  frame="${frame//\"/}"
  if [[ "${frame}" == "camera_color_optical_frame" ]]; then
    echo "PASS: ${topic} frame_id=${frame}"
    return 0
  fi
  echo "FAIL: ${topic} expected frame_id=camera_color_optical_frame, got ${frame:-none}" >&2
  return 1
}

check_perception_diagnostics() {
  local sample
  if ! sample="$(timeout 10s ros2 topic echo --once --timeout 8 /perception/diagnostics \
      --filter "any(s.name == 'perception/pipeline' for s in m.status)" 2>/dev/null)"; then
    echo "FAIL: /perception/diagnostics did not publish a DiagnosticArray" >&2
    return 1
  fi
  local missing=0
  for name in perception/camera_rgb perception/camera_depth perception/camera_info \
    perception/detector perception/tf perception/depth_quality perception/pipeline; do
    if grep -q "name: ${name}" <<<"${sample}"; then
      echo "PASS: /perception/diagnostics contains ${name}"
    else
      echo "FAIL: /perception/diagnostics missing ${name}" >&2
      missing=$((missing + 1))
    fi
  done
  [[ "${missing}" -eq 0 ]]
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

check_camera_tf() {
  local log_file="/tmp/factory_patrol_camera_tf_check.log"
  timeout 5s ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame >"${log_file}" 2>&1 || true
  if grep -q "Translation:" "${log_file}"; then
    echo "PASS: TF base_link -> camera_color_optical_frame"
    return 0
  fi
  echo "FAIL: TF base_link -> camera_color_optical_frame not observed within timeout" >&2
  return 1
}

check_geometry_message() {
  local topic="$1"
  local expected_frame="$2"
  local command_timeout=10
  local echo_timeout=8
  if [[ "${DETECTOR_MODE}" == "true" ]]; then
    command_timeout=30
    echo_timeout=28
  fi
  if timeout "${command_timeout}s" ros2 topic echo --once --timeout "${echo_timeout}" "${topic}" \
      --filter "m.header.frame_id == '${expected_frame}' and (m.header.stamp.sec > 0 or m.header.stamp.nanosec > 0) and m.point.x == m.point.x and m.point.y == m.point.y and m.point.z == m.point.z and abs(m.point.x) < 1000000.0 and abs(m.point.y) < 1000000.0 and abs(m.point.z) < 1000000.0" \
      --no-arr >/dev/null 2>&1; then
    echo "PASS: ${topic} is finite, stamped, and frame_id=${expected_frame}"
    return 0
  fi
  echo "FAIL: ${topic} did not produce a finite stamped point in ${expected_frame}" >&2
  return 1
}

check_geometry_marker() {
  if timeout 10s ros2 topic echo --once --timeout 8 /perception/markers \
      --filter "m.header.frame_id == 'map' and (m.header.stamp.sec > 0 or m.header.stamp.nanosec > 0) and m.type == 2 and m.action == 0" \
      --no-arr >/dev/null 2>&1; then
    echo "PASS: /perception/markers publishes a stamped map-frame sphere"
    return 0
  fi
  echo "FAIL: /perception/markers did not produce the expected marker" >&2
  return 1
}

wait_for_node() {
  local node_name="$1"
  local attempt
  for attempt in $(seq 1 20); do
    timeout 3s ros2 node list 2>/dev/null | grep -Fxq "/${node_name}" && return 0
    sleep 0.25
  done
  return 1
}

expect_no_message() {
  local topic="$1"
  if timeout 3s ros2 topic echo --once "${topic}" --no-arr >/dev/null 2>&1; then
    echo "FAIL: ${topic} unexpectedly published a message" >&2
    return 1
  fi
  echo "PASS: ${topic} correctly produced no false output"
}

run_geometry_failure_probe() {
  local mode="$1"
  local node_name="phase2_${mode}_probe"
  local topic_prefix="/perception/validation_${mode}"
  local log_file="/tmp/factory_patrol_${node_name}.log"
  local geometry_node
  geometry_node="$(ros2 pkg prefix robot_perception)/lib/robot_perception/geometry_validation_node"
  local extra_args=()
  if [[ "${mode}" == "tf_failure" ]]; then
    extra_args=(-p target_frame:=phase2_missing_map_frame)
  elif [[ "${mode}" == "missing_input" ]]; then
    extra_args=(
      -p rgb_topic:=/phase2_missing/rgb
      -p depth_topic:=/phase2_missing/depth
      -p camera_info_topic:=/phase2_missing/camera_info
    )
  else
    extra_args=(-p min_depth:=7.0 -p max_depth:=8.0)
  fi

  "${geometry_node}" --ros-args \
    -r __node:="${node_name}" \
    -p use_sim_time:=true \
    -p camera_point_topic:="${topic_prefix}/camera_point" \
    -p map_point_topic:="${topic_prefix}/map_point" \
    -p marker_topic:="${topic_prefix}/marker" \
    -p ground_truth.enabled:=false \
    "${extra_args[@]}" >"${log_file}" 2>&1 &
  local probe_pid=$!

  if ! wait_for_node "${node_name}"; then
    echo "FAIL: ${node_name} did not start" >&2
    kill "${probe_pid}" 2>/dev/null || true
    wait "${probe_pid}" 2>/dev/null || true
    return 1
  fi

  local probe_failed=0
  if [[ "${mode}" == "tf_failure" ]]; then
    check_geometry_message "${topic_prefix}/camera_point" "camera_color_optical_frame" || probe_failed=1
    expect_no_message "${topic_prefix}/map_point" || probe_failed=1
  else
    expect_no_message "${topic_prefix}/camera_point" || probe_failed=1
    expect_no_message "${topic_prefix}/map_point" || probe_failed=1
  fi
  if ! kill -0 "${probe_pid}" 2>/dev/null; then
    echo "FAIL: ${node_name} exited during failure handling" >&2
    probe_failed=1
  else
    echo "PASS: ${node_name} remained alive"
  fi
  kill "${probe_pid}" 2>/dev/null || true
  wait "${probe_pid}" 2>/dev/null || true

  if [[ "${mode}" == "tf_failure" ]]; then
    grep -q "map output suppressed" "${log_file}" || probe_failed=1
  elif [[ "${mode}" == "invalid_depth" ]]; then
    grep -q "insufficient_valid_depth" "${log_file}" || probe_failed=1
  fi
  return "${probe_failed}"
}

echo "[factory-topic-check] Checking Factory Patrol runtime topics..."
failures=0
for topic in "${TOPICS[@]}"; do
  if wait_for_topic "${topic}"; then
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
echo "[factory-topic-check] Validating RGB-D message types and payloads..."
check_topic_type "/camera/color/image_raw" "sensor_msgs/msg/Image" || failures=$((failures + 1))
check_topic_type "/camera/depth/image_raw" "sensor_msgs/msg/Image" || failures=$((failures + 1))
check_topic_type "/camera/color/camera_info" "sensor_msgs/msg/CameraInfo" || failures=$((failures + 1))
check_image_message "/camera/color/image_raw" || failures=$((failures + 1))
check_image_message "/camera/depth/image_raw" || failures=$((failures + 1))
check_camera_info || failures=$((failures + 1))
check_camera_frame "/camera/color/image_raw" || failures=$((failures + 1))
check_camera_frame "/camera/depth/image_raw" || failures=$((failures + 1))
check_camera_frame "/camera/color/camera_info" || failures=$((failures + 1))

if [[ "${PERCEPTION_DIAGNOSTICS_MODE}" == "true" ]]; then
  echo
  echo "[factory-topic-check] Validating Phase 7 perception diagnostics..."
  check_topic_type "/perception/diagnostics" "diagnostic_msgs/msg/DiagnosticArray" || failures=$((failures + 1))
  check_perception_diagnostics || failures=$((failures + 1))
  check_topic_type "/system_health" "robot_interfaces/msg/RobotState" || failures=$((failures + 1))
fi

echo
echo "[factory-topic-check] Validating Phase 2 geometry outputs..."
check_topic_type "/perception/geometry/camera_point" "geometry_msgs/msg/PointStamped" || failures=$((failures + 1))
check_topic_type "/perception/geometry/map_point" "geometry_msgs/msg/PointStamped" || failures=$((failures + 1))
check_topic_type "/perception/markers" "visualization_msgs/msg/Marker" || failures=$((failures + 1))
check_topic_type "/perception/objects_3d" "robot_interfaces_perception/msg/DetectedObject3D" || failures=$((failures + 1))
check_geometry_message "/perception/geometry/camera_point" "camera_color_optical_frame" || failures=$((failures + 1))
check_geometry_message "/perception/geometry/map_point" "map" || failures=$((failures + 1))
check_geometry_marker || failures=$((failures + 1))
if wait_for_node "geometry_validation_node"; then
  echo "PASS: geometry_validation_node is alive"
else
  echo "FAIL: geometry_validation_node is not present" >&2
  failures=$((failures + 1))
fi

if [[ "${DETECTOR_MODE}" == "true" ]]; then
  echo
  echo "[factory-topic-check] Delegating Phase 3 detector-chain validation..."
  bash scripts/check_factory_patrol_detector_runtime.sh || failures=$((failures + 1))
  check_managed_target || failures=$((failures + 1))
fi

if [[ "${VISUAL_INSPECTION_MODE}" == "true" ]]; then
  echo
  echo "[factory-topic-check] Delegating Phase 5 visual inspection validation..."
  bash scripts/check_factory_patrol_visual_inspection_runtime.sh || failures=$((failures + 1))
fi

if [[ "${PERCEPTION_SAFETY_MODE}" == "true" ]]; then
  echo
  echo "[factory-topic-check] Validating Phase 6 perception safety interface..."
  check_topic_type "/perception/safety_event" \
    "robot_interfaces_perception/msg/PerceptionSafetyEvent" || failures=$((failures + 1))
  check_topic_type "/safety/state" "std_msgs/msg/String" || failures=$((failures + 1))
  check_topic_type "/safety/reason" "std_msgs/msg/String" || failures=$((failures + 1))
  if timeout 30s ros2 topic echo --once --timeout 28 /perception/safety_event \
      --filter "m.header.frame_id == 'map' and (m.header.stamp.sec > 0 or m.header.stamp.nanosec > 0) and m.source == 'robot_perception' and m.safety_state in (0, 1, 2) and m.event_type in ('CLEAR', 'PERSON_NEAR', 'PERSON_TOO_CLOSE', 'PERSON_IN_DANGER_ZONE')" \
      --no-arr >/dev/null 2>&1; then
    echo "PASS: /perception/safety_event publishes valid stamped semantic safety data"
  else
    echo "FAIL: /perception/safety_event did not publish a valid event" >&2
    failures=$((failures + 1))
  fi
fi

echo
if [[ "${DETECTOR_MODE}" == "true" ]]; then
  echo "[factory-topic-check] Synthetic geometry failure probes are covered by the default-mode run."
else
  echo "[factory-topic-check] Validating geometry failure handling..."
  run_geometry_failure_probe "tf_failure" || failures=$((failures + 1))
  run_geometry_failure_probe "invalid_depth" || failures=$((failures + 1))
  run_geometry_failure_probe "missing_input" || failures=$((failures + 1))
fi

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
check_camera_tf || failures=$((failures + 1))

if [[ "${failures}" -eq 0 ]]; then
  echo "[factory-topic-check] PASS: all expected Factory Patrol topics are present."
  exit 0
fi

echo "[factory-topic-check] FAIL: ${failures} validation check(s) failed." >&2
exit 1
