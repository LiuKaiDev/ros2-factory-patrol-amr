#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
CONFIG_FILE="${FACTORY_PATROL_BENCHMARK_CONFIG:-${REPO_ROOT}/src/robot_experiments/config/factory_patrol_benchmark.yaml}"
RESULTS_DIR="${1:-${REPO_ROOT}/src/robot_experiments/results}"
STAMP="$(date +%Y%m%d_%H%M%S)"
FINAL_JSON="${RESULTS_DIR}/factory_patrol_phase8_${STAMP}.json"
FINAL_CSV="${RESULTS_DIR}/factory_patrol_phase8_${STAMP}.csv"
RUN_DIR="$(mktemp -d /tmp/factory_patrol_phase8.XXXXXX)"
BENCHMARK_DOMAIN_ID="${FACTORY_PATROL_BENCHMARK_DOMAIN_ID:-140}"
BENCHMARK_GZ_PARTITION="${FACTORY_PATROL_BENCHMARK_GZ_PARTITION:-factory_patrol_phase8_${BENCHMARK_DOMAIN_ID}}"
STACK_PID=""
EXTRA_PID=""
PROFILE_SEQUENCE=0
PROFILE_DOMAINS=()

safe_source() {
  set +u
  # shellcheck disable=SC1090
  source "$1"
  set -u
}

fail() {
  echo "FAIL: $*" >&2
  exit 1
}

config_value() {
  python3 - "${CONFIG_FILE}" "$1" <<'PY'
import sys
import yaml

with open(sys.argv[1], encoding="utf-8") as stream:
    value = yaml.safe_load(stream)
for key in sys.argv[2].split("."):
    value = value[key]
if isinstance(value, bool):
    print(str(value).lower())
elif isinstance(value, (dict, list)):
    import json
    print(json.dumps(value, separators=(",", ":")))
else:
    print(value)
PY
}

stop_process_tree() {
  local pid="$1"
  [[ -n "${pid}" ]] || return 0
  if kill -0 -- "-${pid}" 2>/dev/null; then
    # Every background stack has its own session, so shutdown reaches Gazebo
    # even if WSL reparents it before ros2 launch exits.
    kill -INT "${pid}" 2>/dev/null || true
    for _ in $(seq 1 60); do
      kill -0 -- "-${pid}" 2>/dev/null || break
      sleep 0.25
    done
    if kill -0 -- "-${pid}" 2>/dev/null; then
      kill -TERM -- "-${pid}" 2>/dev/null || true
      for _ in $(seq 1 20); do
        kill -0 -- "-${pid}" 2>/dev/null || break
        sleep 0.25
      done
    fi
    if kill -0 -- "-${pid}" 2>/dev/null; then
      kill -KILL -- "-${pid}" 2>/dev/null || true
    fi
  fi
  wait "${pid}" 2>/dev/null || true
}

stop_stack() {
  stop_process_tree "${EXTRA_PID}"
  EXTRA_PID=""
  stop_process_tree "${STACK_PID}"
  STACK_PID=""
  sleep 2
}

cleanup() {
  stop_stack
  if [[ "${KEEP_FACTORY_PATROL_BENCHMARK_LOGS:-false}" != "true" ]]; then
    find "${RUN_DIR}" -depth -delete
  else
    echo "Benchmark scratch retained: ${RUN_DIR}"
  fi
}
trap cleanup EXIT

start_stack() {
  local name="$1"
  shift
  PROFILE_SEQUENCE=$((PROFILE_SEQUENCE + 1))
  export ROS_DOMAIN_ID=$((BENCHMARK_DOMAIN_ID + PROFILE_SEQUENCE - 1))
  export GZ_PARTITION="${BENCHMARK_GZ_PARTITION}_${PROFILE_SEQUENCE}_${name}"
  PROFILE_DOMAINS+=("${name}:${ROS_DOMAIN_ID}:${GZ_PARTITION}")
  echo "[phase8] isolation profile=${name} ROS_DOMAIN_ID=${ROS_DOMAIN_ID} GZ_PARTITION=${GZ_PARTITION}"
  setsid ros2 launch robot_bringup factory_patrol_demo.launch.py \
    gui:=false use_rviz:=false use_mission_runner:=false autostart_mission:=false \
    use_localization_health:=false "$@" >"${RUN_DIR}/${name}_stack.log" 2>&1 &
  STACK_PID=$!
}

run_probe() {
  local name="$1"
  shift
  local probe_log="${RUN_DIR}/${name}_probe.log"
  if ! ros2 run robot_experiments factory_patrol_benchmark "$@" >"${probe_log}" 2>&1; then
    echo "FAIL: ${name} benchmark profile failed" >&2
    sed 's/^/  probe: /' "${probe_log}" >&2
    if [[ -f "${RUN_DIR}/${name}_stack.log" ]]; then
      tail -n 80 "${RUN_DIR}/${name}_stack.log" | sed 's/^/  stack: /' >&2
    fi
    return 1
  fi
  sed 's/^/  /' "${probe_log}"
}

cd "${REPO_ROOT}"
[[ -f /opt/ros/jazzy/setup.bash ]] || fail "/opt/ros/jazzy/setup.bash is unavailable"
safe_source /opt/ros/jazzy/setup.bash
[[ -f install/setup.bash ]] || fail "install/setup.bash is unavailable; build the workspace first"
safe_source install/setup.bash
[[ -f "${CONFIG_FILE}" ]] || fail "benchmark config is missing: ${CONFIG_FILE}"
command -v setsid >/dev/null || fail "setsid is required for isolated launch cleanup"
command -v ros2 >/dev/null || fail "ros2 is unavailable after sourcing the workspace"
export ROS_DOMAIN_ID="${BENCHMARK_DOMAIN_ID}"
export GZ_PARTITION="${BENCHMARK_GZ_PARTITION}"

BRANCH="$(git branch --show-current)"
[[ "${BRANCH}" == "feature/visual-perception-upgrade" ]] ||
  fail "benchmark must run on feature/visual-perception-upgrade, got ${BRANCH}"
git merge-base --is-ancestor bfb59f9 HEAD || fail "required Phase 7 commit bfb59f9 is not in history"

bash scripts/check_factory_patrol_assets.sh
bash scripts/prepare_phase3_detector_model.sh

MODEL_PATH="${XDG_CACHE_HOME:-${HOME}/.cache}/robot_perception/models/object_detection_yolox_2022nov.onnx"
[[ -s "${MODEL_PATH}" ]] || fail "detector model is missing after prerequisite validation"

DETECTOR_WARMUP="$(config_value benchmark.detector.warmup_samples)"
DETECTOR_SAMPLES="$(config_value benchmark.detector.measured_samples)"
DETECTOR_RATE="$(config_value benchmark.detector.max_inference_rate_hz)"
DETECTOR_TIMEOUT="$(config_value benchmark.detector.timeout_sec)"
GEOMETRY_SAMPLES="$(config_value benchmark.geometry.samples_per_condition)"
GEOMETRY_SETTLE="$(config_value benchmark.geometry.settle_samples)"
GEOMETRY_TIMEOUT="$(config_value benchmark.geometry.timeout_sec)"
GEOMETRY_CONDITIONS="$(config_value benchmark.geometry.conditions)"
MISSION_RUNS="${FACTORY_PATROL_MISSION_RUNS:-$(config_value benchmark.mission.runs)}"
MISSION_TIMEOUT="$(config_value benchmark.mission.timeout_sec)"
MISSION_DUPLICATE_WINDOW="$(config_value benchmark.mission.duplicate_observation_sec)"
SAFETY_RUNS="${FACTORY_PATROL_SAFETY_RUNS:-$(config_value benchmark.safety.runs)}"
SAFETY_TIMEOUT="$(config_value benchmark.safety.timeout_sec)"
INVALID_CASES="$(config_value benchmark.invalid_depth.cases)"
INVALID_SETTLE="$(config_value benchmark.invalid_depth.settle_sec)"
INVALID_TIMEOUT="$(config_value benchmark.invalid_depth.timeout_sec)"

mkdir -p "${RESULTS_DIR}"

echo "[phase8] detector latency and stationary-target stability"
DETECTOR_OUTPUT="${RUN_DIR}/detector.json"
start_stack detector \
  use_nav2:=false use_geometry_validation:=true use_detector:=true \
  geometry_input_mode:=detector use_visual_inspection:=false \
  use_perception_safety:=false use_perception_diagnostics:=true \
  use_perception_system_monitor:=true \
  detector_model_path:="${MODEL_PATH}" \
  perception_debug_image:=false \
  perception_max_inference_rate_hz:="${DETECTOR_RATE}"
run_probe detector profile --profile detector --output "${DETECTOR_OUTPUT}" \
  --warmup "${DETECTOR_WARMUP}" --samples "${DETECTOR_SAMPLES}" \
  --timeout "${DETECTOR_TIMEOUT}"
stop_stack

echo "[phase8] deterministic multi-range geometry"
GEOMETRY_OUTPUT="${RUN_DIR}/geometry.json"
start_stack geometry \
  use_nav2:=false use_geometry_validation:=true use_detector:=false \
  geometry_input_mode:=synthetic use_visual_inspection:=false \
  use_perception_safety:=false use_perception_diagnostics:=false \
  use_perception_system_monitor:=false
run_probe geometry profile --profile geometry --output "${GEOMETRY_OUTPUT}" \
  --samples "${GEOMETRY_SAMPLES}" --settle-samples "${GEOMETRY_SETTLE}" \
  --conditions "${GEOMETRY_CONDITIONS}" --timeout "${GEOMETRY_TIMEOUT}"
stop_stack

MISSION_OUTPUTS=()
for run_index in $(seq 1 "${MISSION_RUNS}"); do
  echo "[phase8] visual inspection mission ${run_index}/${MISSION_RUNS}"
  mission_output="${RUN_DIR}/mission_${run_index}.json"
  MISSION_OUTPUTS+=("${mission_output}")
  start_stack "mission_${run_index}" \
    use_nav2:=true use_geometry_validation:=true use_detector:=true \
    geometry_input_mode:=detector use_visual_inspection:=true \
    use_perception_safety:=false use_perception_diagnostics:=true \
    use_perception_system_monitor:=true visual_inspection_start_delay:=10.0 \
    detector_model_path:="${MODEL_PATH}" perception_debug_image:=false \
    perception_max_inference_rate_hz:="${DETECTOR_RATE}" \
    perception_tracking_params:="${REPO_ROOT}/src/robot_perception/config/tracking_phase5_validation.yaml" \
    visual_inspection_params:="${REPO_ROOT}/src/robot_tasks/config/visual_inspection_phase5_validation.yaml"
  run_probe "mission_${run_index}" profile --profile mission \
    --output "${mission_output}" --run-index "${run_index}" \
    --timeout "${MISSION_TIMEOUT}" --duplicate-window "${MISSION_DUPLICATE_WINDOW}"
  stop_stack
done

echo "[phase8] repeated Safety Gate response"
SAFETY_OUTPUT="${RUN_DIR}/safety.json"
start_stack safety \
  use_nav2:=true use_geometry_validation:=true use_detector:=true \
  geometry_input_mode:=detector use_visual_inspection:=false \
  use_perception_safety:=true use_perception_diagnostics:=true \
  use_perception_system_monitor:=true \
  detector_model_path:="${MODEL_PATH}" perception_debug_image:=false \
  perception_max_inference_rate_hz:="${DETECTOR_RATE}" \
  perception_tracking_params:="${REPO_ROOT}/src/robot_perception/config/tracking_phase6_validation.yaml"
run_probe safety profile --profile safety --output "${SAFETY_OUTPUT}" \
  --samples "${SAFETY_RUNS}" --timeout "${SAFETY_TIMEOUT}"
stop_stack

echo "[phase8] deterministic invalid-depth rejection"
INVALID_OUTPUT="${RUN_DIR}/invalid_depth.json"
start_stack invalid_depth \
  use_nav2:=false use_geometry_validation:=false use_detector:=false \
  use_visual_inspection:=false use_perception_safety:=false \
  use_perception_diagnostics:=false use_perception_system_monitor:=false
GEOMETRY_NODE="$(ros2 pkg prefix robot_perception)/lib/robot_perception/geometry_validation_node"
setsid "${GEOMETRY_NODE}" --ros-args \
  -r __node:=phase8_invalid_depth_geometry \
  -p use_sim_time:=true -p geometry_input_mode:=synthetic \
  -p rgb_topic:=/phase8_invalid/rgb \
  -p depth_topic:=/phase8_invalid/depth \
  -p camera_info_topic:=/phase8_invalid/camera_info \
  -p camera_point_topic:=/phase8_invalid/camera_point \
  -p map_point_topic:=/phase8_invalid/map_point \
  -p marker_topic:=/phase8_invalid/markers \
  -p objects_3d_topic:=/phase8_invalid/objects_3d \
  -p inspection.event_topic:=/phase8_invalid/events \
  -p safety.event_topic:=/phase8_invalid/safety_event \
  -p synthetic_bbox.center_u:=10.0 -p synthetic_bbox.center_v:=10.0 \
  -p synthetic_bbox.width:=8.0 -p synthetic_bbox.height:=8.0 \
  -p ground_truth.enabled:=false >"${RUN_DIR}/invalid_depth_geometry.log" 2>&1 &
EXTRA_PID=$!
run_probe invalid_depth profile --profile invalid_depth --output "${INVALID_OUTPUT}" \
  --samples "${INVALID_CASES}" --settle-sec "${INVALID_SETTLE}" \
  --timeout "${INVALID_TIMEOUT}"
stop_stack

GIT_COMMIT="$(git rev-parse HEAD)"
GIT_DIRTY=false
[[ -z "$(git status --porcelain)" ]] || GIT_DIRTY=true
DIFF_SHA256="$(git diff --no-ext-diff | sha256sum | awk '{print $1}')"
PROFILE_DOMAIN_RECORD="$(IFS=,; echo "${PROFILE_DOMAINS[*]}")"
METADATA="$(python3 - "${CONFIG_FILE}" "${GIT_COMMIT}" "${BRANCH}" "${GIT_DIRTY}" \
  "${DIFF_SHA256}" "${ROS_DISTRO:-unknown}" "${MODEL_PATH}" "${BENCHMARK_DOMAIN_ID}" \
  "${BENCHMARK_GZ_PARTITION}" "${PROFILE_DOMAIN_RECORD}" <<'PY'
import json
import sys
import yaml

with open(sys.argv[1], encoding="utf-8") as stream:
    config = yaml.safe_load(stream)["benchmark"]
print(json.dumps({
    "git_commit": sys.argv[2],
    "git_branch": sys.argv[3],
    "working_tree_dirty": sys.argv[4] == "true",
    "working_tree_diff_sha256": sys.argv[5],
    "ros_distro": sys.argv[6],
    "ros_domain_id_base": int(sys.argv[8]),
    "gz_partition_base": sys.argv[9],
    "profile_transport_isolation": sys.argv[10].split(","),
    "world": config["world"],
    "benchmark_profile": config["profile"],
    "headless": config["headless"],
    "gazebo_gui": False,
    "rviz": False,
    "use_sim_time": config["use_sim_time"],
    "detector": {
        "backend": "opencv_yolox",
        "model": sys.argv[7],
        "requested_device": "auto",
        "expected_device": "cpu",
        "input_size": 640,
        "confidence_threshold": 0.45,
        "max_inference_rate_hz": config["detector"]["max_inference_rate_hz"],
    },
    "camera": {"resolution": [640, 480], "rate_hz": 15, "frame": "camera_color_optical_frame"},
    "tracking": {
        "confirm_frames": 3,
        "lost_frames": 5,
        "max_match_distance_m": 1.0,
        "ema_alpha": 0.4,
        "processed_cooldown_sec": 120.0,
    },
    "safety": {
        "stop_distance_m": config["safety"]["stop_distance_m"],
        "slow_distance_m": config["safety"]["slow_distance_m"],
        "speed_limit_linear_mps": config["safety"]["speed_limit_linear_mps"],
        "speed_limit_angular_radps": config["safety"]["speed_limit_angular_radps"],
    },
    "standoff_distance_m": config["mission"]["standoff_distance_m"],
    "configured_samples": {
        "detector_warmup": config["detector"]["warmup_samples"],
        "detector_measured": config["detector"]["measured_samples"],
        "geometry_per_condition": config["geometry"]["samples_per_condition"],
        "mission_runs": int(__import__("os").environ.get("FACTORY_PATROL_MISSION_RUNS", config["mission"]["runs"])),
        "safety_runs": int(__import__("os").environ.get("FACTORY_PATROL_SAFETY_RUNS", config["safety"]["runs"])),
        "invalid_depth_cases": config["invalid_depth"]["cases"],
    },
}, separators=(",", ":")))
PY
)"

MERGE_ARGS=(
  merge --input "${DETECTOR_OUTPUT}" --input "${GEOMETRY_OUTPUT}"
  --input "${SAFETY_OUTPUT}" --input "${INVALID_OUTPUT}"
  --output "${FINAL_JSON}" --csv "${FINAL_CSV}" --metadata "${METADATA}"
)
for mission_output in "${MISSION_OUTPUTS[@]}"; do
  MERGE_ARGS+=(--input "${mission_output}")
done
ros2 run robot_experiments factory_patrol_benchmark "${MERGE_ARGS[@]}"

echo "[phase8] PASS: complete benchmark suite"
echo "JSON=${FINAL_JSON}"
echo "CSV=${FINAL_CSV}"
