#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

safe_source_setup() {
  local setup_file="$1"
  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
}

if [[ -f install/setup.bash ]]; then
  safe_source_setup install/setup.bash
fi

fail() {
  echo "FAIL: $*" >&2
  exit 1
}

topic_type_is() {
  local topic="$1"
  local expected="$2"
  local actual
  actual="$(timeout 5s ros2 topic type "${topic}" 2>/dev/null || true)"
  [[ "${actual}" == "${expected}" ]] ||
    fail "${topic} expected ${expected}, got ${actual:-none}"
  echo "PASS: ${topic} type=${actual}"
}

topic_publishers() {
  local topic="$1"
  timeout 5s ros2 topic info "${topic}" -v 2>/dev/null |
    awk -F': ' '/Publisher count:/ {print $2; exit}'
}

sample_map_pose() {
  local output_file="$1"
  local pose
  pose="$(timeout 30s ros2 run tf2_ros tf2_echo map base_link -r 10 \
    --ros-args -p use_sim_time:=true 2>/dev/null | awk '
    /- Translation:/ {
      line=$0
      gsub(/.*\[/, "", line)
      gsub(/\].*/, "", line)
      gsub(/,/, "", line)
      split(line, translation, /[[:space:]]+/)
      tx=translation[1]; ty=translation[2]
    }
    /- Rotation: in Quaternion/ {
      line=$0
      gsub(/.*\[/, "", line)
      gsub(/\].*/, "", line)
      gsub(/,/, "", line)
      split(line, rotation, /[[:space:]]+/)
      qz=rotation[3]; qw=rotation[4]
      if (tx != "" && ty != "" && qz != "" && qw != "") {
        yaw=atan2(2*qw*qz, 1-2*qz*qz)
        printf "%.9f %.9f %.9f\n", tx, ty, yaw
        exit
      }
    }')" || true
  [[ -n "${pose}" ]] || fail "could not sample map -> base_link TF within 30 s"
  printf '%s\n' "${pose}" >"${output_file}"
}

echo "[visual-inspection-check] Validating visual inspection event and mission chain..."
topic_type_is /perception/events robot_interfaces_perception/msg/PerceptionEvent
topic_type_is /inspection/observation_pose geometry_msgs/msg/PoseStamped
topic_type_is /inspection/status std_msgs/msg/String
topic_type_is /navigate_sequence/current_goal geometry_msgs/msg/PoseStamped

perception_allowlist="$(timeout 5s ros2 param get /geometry_validation_node inspection.allowed_classes 2>/dev/null || true)"
task_allowlist="$(timeout 5s ros2 param get /visual_inspection_task_node inspection.allowed_classes 2>/dev/null || true)"
[[ "${perception_allowlist}" == *person* ]] ||
  fail "visual inspection validation requires the explicit perception person allowlist profile"
[[ "${task_allowlist}" == *person* ]] ||
  fail "visual inspection validation requires the explicit task person allowlist profile"
echo "PASS: static person fixture is explicitly enabled only for this validation profile"

event_log="$(mktemp)"
processed_log="$(mktemp)"
status_log="$(mktemp)"
mission_status_log="$(mktemp)"
mission_event_log="$(mktemp)"
observation_pose_log="$(mktemp)"
nav_goal_log="$(mktemp)"
map_pose_after="$(mktemp)"
odom_path="$(mktemp)"
event_echo_pid=""
processed_echo_pid=""
status_echo_pid=""
mission_status_echo_pid=""
observation_echo_pid=""
nav_goal_echo_pid=""
cleanup() {
  for pid in "${event_echo_pid}" "${processed_echo_pid}" "${status_echo_pid}" \
    "${mission_status_echo_pid}" "${observation_echo_pid}" "${nav_goal_echo_pid}"; do
    [[ -n "${pid}" ]] && kill "${pid}" 2>/dev/null || true
  done
  rm -f "${event_log}" "${processed_log}" "${status_log}" \
    "${mission_status_log}" "${mission_event_log}" "${observation_pose_log}" \
    "${nav_goal_log}" "${map_pose_after}" "${odom_path}"
}
trap cleanup EXIT

PYTHONUNBUFFERED=1 timeout 110s ros2 topic echo /perception/events --timeout 107 \
  --qos-reliability reliable --qos-durability transient_local \
  --filter "m.class_name == 'person' and m.target_pose.header.frame_id == 'map' and m.target_id > 0 and m.event_type in ('INSPECTION_REQUIRED', 'INSPECTION_COMPLETED')" \
  --no-arr >"${event_log}" 2>/dev/null &
event_echo_pid=$!
timeout 100s ros2 topic echo --once /perception/objects_3d --timeout 97 \
  --filter "m.class_name == 'person' and m.header.frame_id == 'map' and m.tracking_state == 3" \
  --no-arr >"${processed_log}" 2>/dev/null &
processed_echo_pid=$!
PYTHONUNBUFFERED=1 timeout 100s ros2 topic echo --once /inspection/status --timeout 97 \
  --qos-reliability reliable --qos-durability transient_local \
  --filter "m.data.startswith('SUCCEEDED:')" \
  >"${status_log}" 2>/dev/null &
status_echo_pid=$!
PYTHONUNBUFFERED=1 timeout 110s ros2 topic echo /inspection/status --timeout 107 \
  --qos-reliability reliable --qos-durability transient_local \
  --filter "m.data.startswith(('REQUESTED:', 'NAVIGATING:', 'SUCCEEDED:', 'FAILED:'))" \
  >"${mission_status_log}" 2>/dev/null &
mission_status_echo_pid=$!
timeout 100s ros2 topic echo --once /inspection/observation_pose --timeout 97 \
  --qos-reliability reliable --qos-durability transient_local \
  --filter "m.header.frame_id == 'map' and m.pose.position.x == m.pose.position.x and m.pose.position.y == m.pose.position.y and m.pose.orientation.w == m.pose.orientation.w" \
  --no-arr >"${observation_pose_log}" 2>/dev/null &
observation_echo_pid=$!
timeout 100s ros2 topic echo --once /navigate_sequence/current_goal --timeout 97 \
  --qos-reliability reliable --qos-durability transient_local \
  --filter "m.header.frame_id == 'map'" --no-arr \
  >"${nav_goal_log}" 2>/dev/null &
nav_goal_echo_pid=$!

wait "${processed_echo_pid}" || fail "target did not reach PROCESSED state"
processed_echo_pid=""
wait "${status_echo_pid}" || fail "visual inspection task did not report SUCCEEDED"
status_echo_pid=""
wait "${observation_echo_pid}" || fail "no finite map-frame observation pose was published"
observation_echo_pid=""
wait "${nav_goal_echo_pid}" || fail "existing navigation adapter did not publish the visual inspection goal"
nav_goal_echo_pid=""
grep -q '^data:.*SUCCEEDED:' "${status_log}" ||
  fail "visual inspection task did not report SUCCEEDED"
grep -q 'frame_id: map' "${observation_pose_log}" ||
  fail "no finite map-frame observation pose was published"
grep -q 'frame_id: map' "${nav_goal_log}" ||
  fail "existing navigation adapter did not publish the visual inspection goal"
# Immediate duplicate suppression is checked inside the validation profile's 120-second cooldown.
sleep 5
kill "${event_echo_pid}" 2>/dev/null || true
wait "${event_echo_pid}" 2>/dev/null || true
event_echo_pid=""
kill "${mission_status_echo_pid}" 2>/dev/null || true
wait "${mission_status_echo_pid}" 2>/dev/null || true
mission_status_echo_pid=""

sample_map_pose "${map_pose_after}"
timeout 10s ros2 topic echo --once /odom/path --timeout 8 --field poses \
  >"${odom_path}" 2>/dev/null || fail "could not sample accumulated odometry path"

processed_target_id="$(awk '/^target_id:/ {print $2; exit}' "${processed_log}")"
[[ -n "${processed_target_id}" ]] || fail "PROCESSED target ID is missing"
target_id="${processed_target_id}"
read -r required_count completed_count <<EOF
$(awk -v expected_id="${target_id}" '
  function count_event() {
    if (target_id == expected_id && event_type == "INSPECTION_REQUIRED") required++
    if (target_id == expected_id && event_type == "INSPECTION_COMPLETED") completed++
    target_id=""; event_type=""
  }
  /^---$/ {count_event(); next}
  /^target_id:/ {target_id=$2}
  /^event_type:/ {event_type=$2}
  END {count_event(); print required+0, completed+0}' "${event_log}")
EOF
[[ "${required_count}" -eq 1 ]] ||
  fail "expected one INSPECTION_REQUIRED event for target ${target_id}, observed ${required_count}"
echo "PASS: target ${target_id} emitted INSPECTION_REQUIRED exactly once"

[[ "${completed_count}" -eq 1 ]] ||
  fail "expected one INSPECTION_COMPLETED event for target ${target_id}, observed ${completed_count}"
echo "PASS: target ${target_id} completion feedback was published exactly once"

awk -v RS='---' -v expected_id="${target_id}" '
  $0 ~ "(^|\\n)target_id: " expected_id "(\\n|$)" &&
  $0 ~ "event_type: INSPECTION_REQUIRED" {print; exit}' \
  "${event_log}" >"${mission_event_log}"
[[ -s "${mission_event_log}" ]] ||
  fail "could not associate target ${target_id} with its inspection event"

accepted_count="$(grep -c '^data:.*NAVIGATING: inspection goal accepted' "${mission_status_log}" || true)"
[[ "${accepted_count}" -eq 1 ]] ||
  fail "expected exactly one accepted inspection mission, observed ${accepted_count}"
echo "PASS: exactly one inspection mission was accepted"

target_class="$(awk '/^class_name:/ {print $2; exit}' "${mission_event_log}")"
target_x="$(awk '/^[[:space:]]+x:/ {print $2; exit}' "${mission_event_log}")"
target_y="$(awk '/^[[:space:]]+y:/ {print $2; exit}' "${mission_event_log}")"
observation_x="$(awk '/^[[:space:]]+x:/ {print $2; exit}' "${observation_pose_log}")"
observation_y="$(awk '/^[[:space:]]+y:/ {print $2; exit}' "${observation_pose_log}")"
observation_qz="$(awk '/orientation:/ {orientation=1; next} orientation && /^[[:space:]]+z:/ {print $2; exit}' "${observation_pose_log}")"
observation_qw="$(awk '/orientation:/ {orientation=1; next} orientation && /^[[:space:]]+w:/ {print $2; exit}' "${observation_pose_log}")"
nav_goal_x="$(awk '/^[[:space:]]+x:/ {print $2; exit}' "${nav_goal_log}")"
nav_goal_y="$(awk '/^[[:space:]]+y:/ {print $2; exit}' "${nav_goal_log}")"
read -r final_x final_y final_yaw <"${map_pose_after}"
path_start_x="$(awk '/position:/ {position=1; next} /orientation:/ {position=0} position && /^[[:space:]]+x:/ {print $2; exit}' "${odom_path}")"
path_start_y="$(awk '/position:/ {position=1; next} /orientation:/ {position=0} position && /^[[:space:]]+y:/ {print $2; exit}' "${odom_path}")"
path_end_x="$(awk '/position:/ {position=1; next} /orientation:/ {position=0} position && /^[[:space:]]+x:/ {value=$2} END {print value}' "${odom_path}")"
path_end_y="$(awk '/position:/ {position=1; next} /orientation:/ {position=0} position && /^[[:space:]]+y:/ {value=$2} END {print value}' "${odom_path}")"

read -r planned_distance final_distance final_goal_error movement_distance yaw_error nav_goal_error <<EOF
$(awk -v tx="${target_x}" -v ty="${target_y}" -v ox="${observation_x}" \
  -v oy="${observation_y}" -v fx="${final_x}" -v fy="${final_y}" \
  -v ix="${path_start_x}" -v iy="${path_start_y}" \
  -v ex="${path_end_x:-${final_x}}" -v ey="${path_end_y:-${final_y}}" \
  -v qz="${observation_qz}" -v qw="${observation_qw}" \
  -v ngx="${nav_goal_x}" -v ngy="${nav_goal_y}" '
  BEGIN {
    pd = sqrt((ox-tx)^2 + (oy-ty)^2)
    fd = sqrt((fx-tx)^2 + (fy-ty)^2)
    fg = sqrt((fx-ox)^2 + (fy-oy)^2)
    md = sqrt((ex-ix)^2 + (ey-iy)^2)
    ng = sqrt((ngx-ox)^2 + (ngy-oy)^2)
    planned_yaw = atan2(ty-oy, tx-ox)
    pose_yaw = atan2(2*qw*qz, 1-2*qz*qz)
    error = pose_yaw - planned_yaw
    pi = atan2(0, -1)
    while (error > pi) error -= 2*pi
    while (error < -pi) error += 2*pi
    printf "%.6f %.6f %.6f %.6f %.6f %.6f\n", pd, fd, fg, md, error, ng
  }')
EOF

awk -v value="${planned_distance}" 'BEGIN {exit !(value >= 1.15 && value <= 1.25)}' ||
  fail "planned target distance ${planned_distance} m does not match 1.2 m standoff"
awk -v value="${yaw_error}" 'BEGIN {if (value < 0) value=-value; exit !(value <= 0.02)}' ||
  fail "observation yaw does not face target; error=${yaw_error} rad"
awk -v value="${movement_distance}" 'BEGIN {exit !(value > 0.20)}' ||
  fail "robot did not move measurably; displacement=${movement_distance} m"
awk -v value="${nav_goal_error}" 'BEGIN {exit !(value <= 0.001)}' ||
  fail "navigation adapter goal differs from observation pose by ${nav_goal_error} m"
awk -v value="${final_goal_error}" 'BEGIN {exit !(value <= 0.50)}' ||
  fail "robot finished ${final_goal_error} m from the observation pose"

read -r required_sec required_nanosec completed_sec completed_nanosec <<EOF
$(awk -v expected_id="${target_id}" '
  function capture() {
    if (target_id != expected_id) return
    if (event_type == "INSPECTION_REQUIRED") {
      required_sec=sec; required_nanosec=nanosec
    }
    if (event_type == "INSPECTION_COMPLETED") {
      completed_sec=sec; completed_nanosec=nanosec
    }
  }
  /^---$/ {capture(); sec=""; nanosec=""; target_id=""; event_type=""; next}
  /^    sec:/ && sec == "" {sec=$2}
  /^    nanosec:/ && nanosec == "" {nanosec=$2}
  /^target_id:/ {target_id=$2}
  /^event_type:/ {event_type=$2}
  END {capture(); print required_sec, required_nanosec, completed_sec, completed_nanosec}' "${event_log}")
EOF
mission_duration="$(awk -v rs="${required_sec}" -v rn="${required_nanosec}" \
  -v cs="${completed_sec}" -v cn="${completed_nanosec}" \
  'BEGIN {printf "%.3f", (cs-rs) + (cn-rn)/1000000000.0}')"

perception_cmd_vel_hits="$(
  for node in /detector_node /geometry_validation_node; do
    if timeout 5s ros2 node list 2>/dev/null | grep -Fxq "${node}"; then
      ros2 node info "${node}" 2>/dev/null |
        awk '/Publishers:/,/Service Servers:/' |
        grep -E '/(cmd_vel|nav2_cmd_vel):' || true
    fi
  done
)"
[[ -z "${perception_cmd_vel_hits}" ]] ||
  fail "perception node publishes a velocity topic: ${perception_cmd_vel_hits}"
echo "PASS: no perception node publishes /cmd_vel or /nav2_cmd_vel"

[[ "$(topic_publishers /nav2_cmd_vel)" -ge 1 ]] || fail "/nav2_cmd_vel has no Nav2 publisher"
[[ "$(topic_publishers /cmd_vel)" -ge 1 ]] || fail "/cmd_vel has no mux/gate publisher"
timeout 5s ros2 node list | grep -Fxq /cmd_vel_mux_node ||
  fail "existing combined cmd_vel mux/Safety Gate node is not running"
echo "PASS: Nav2 command remains /nav2_cmd_vel -> existing mux/Safety Gate -> /cmd_vel"

echo "[visual-inspection-check] target_id=${target_id}"
echo "[visual-inspection-check] target_class=${target_class} (explicit static validation fixture)"
echo "[visual-inspection-check] event_timestamp=${required_sec}.${required_nanosec} sim seconds"
echo "[visual-inspection-check] target_map_pose=(${target_x}, ${target_y})"
echo "[visual-inspection-check] observation_pose=(${observation_x}, ${observation_y})"
echo "[visual-inspection-check] configured_standoff_distance=1.2 m"
echo "[visual-inspection-check] planned_target_distance=${planned_distance} m"
echo "[visual-inspection-check] nav2_goal_acceptance=accepted"
echo "[visual-inspection-check] nav2_result=SUCCEEDED"
echo "[visual-inspection-check] mission_elapsed=${mission_duration} sim seconds"
echo "[visual-inspection-check] final_robot_pose=(${final_x}, ${final_y}, yaw=${final_yaw})"
echo "[visual-inspection-check] final_observation_pose_error=${final_goal_error} m"
echo "[visual-inspection-check] actual_final_robot_target_distance=${final_distance} m"
echo "[visual-inspection-check] robot_displacement=${movement_distance} m"
echo "[visual-inspection-check] PASS: visual inspection mission succeeded and target became PROCESSED"
