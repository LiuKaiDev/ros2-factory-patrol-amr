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
  pass "found ${file}"
}

require_grep() {
  local pattern="$1"
  local file="$2"
  local message="$3"
  grep -q "${pattern}" "${ROOT_DIR}/${file}" || fail "${message}"
  pass "${file} contains ${pattern}"
}

require_camera_sensor_grep() {
  local pattern="$1"
  local file="$2"
  local message="$3"
  local sensor_block
  sensor_block="$(sed -n '/<sensor name="rgbd_camera" type="rgbd_camera">/,/<\/sensor>/p' "${ROOT_DIR}/${file}")"
  grep -q "${pattern}" <<<"${sensor_block}" || fail "${message}"
  pass "${file} RGB-D sensor contains ${pattern}"
}

WORLD_FILE="src/robot_simulation/worlds/factory_patrol.sdf"
INDUSTRIAL_WORLD_FILE="src/robot_simulation/worlds/factory_patrol_industrial.sdf"
STATIONS_FILE="src/robot_simulation/config/factory_patrol_stations.yaml"
ZONES_FILE="src/robot_simulation/config/factory_patrol_zones.yaml"
ROUTE_FILE="src/robot_simulation/config/factory_patrol_route.yaml"
MAP_README="src/robot_navigation/maps/factory_patrol_map_README.md"
DEMO_LAUNCH="src/robot_bringup/launch/factory_patrol_demo.launch.py"
RUN_SCRIPT="scripts/run_factory_patrol_demo.sh"
SIM_LAUNCH="src/robot_simulation/launch/sim.launch.py"
ROBOT_XACRO="src/robot_description/urdf/robot.urdf.xacro"
SHOWCASE_RVIZ="src/robot_simulation/rviz/factory_patrol_showcase.rviz"
PERCEPTION_PACKAGE="src/robot_perception"
PERCEPTION_INTERFACES="src/robot_interfaces_perception"
SCENARIO_DOC="docs/simulation_scenarios.md"
ARCH_DOC="docs/architecture.md"
README_FILE="README.md"
EXPERIMENT_DOC="docs/experiment_report.md"
EXPERIMENT_PACKAGE="src/robot_experiments"

for file in "${WORLD_FILE}" "${INDUSTRIAL_WORLD_FILE}" "${STATIONS_FILE}" "${ZONES_FILE}" "${ROUTE_FILE}" \
  "${MAP_README}" "${DEMO_LAUNCH}" "${RUN_SCRIPT}" "${SIM_LAUNCH}" \
  "${ROBOT_XACRO}" "${SHOWCASE_RVIZ}" "${PERCEPTION_PACKAGE}/CMakeLists.txt" \
  "${PERCEPTION_PACKAGE}/package.xml" "${PERCEPTION_PACKAGE}/config/depth.yaml" \
  "${PERCEPTION_PACKAGE}/config/detector.yaml" \
  "${PERCEPTION_PACKAGE}/config/diagnostics.yaml" \
  "${PERCEPTION_PACKAGE}/config/tracking.yaml" \
  "${PERCEPTION_PACKAGE}/config/safety.yaml" \
  "${PERCEPTION_PACKAGE}/config/safety_zones.yaml" \
  "${PERCEPTION_PACKAGE}/config/tracking_phase6_validation.yaml" \
  "${PERCEPTION_PACKAGE}/launch/geometry_validation.launch.py" \
  "${PERCEPTION_PACKAGE}/launch/perception.launch.py" \
  "${PERCEPTION_PACKAGE}/robot_perception/detector_backend.py" \
  "${PERCEPTION_PACKAGE}/robot_perception/detector_node.py" \
  "${PERCEPTION_PACKAGE}/test/test_detection_utils.py" \
  "${PERCEPTION_PACKAGE}/test/depth_projector_test.cpp" \
  "${PERCEPTION_PACKAGE}/test/target_manager_test.cpp" \
  "${PERCEPTION_PACKAGE}/test/inspection_event_policy_test.cpp" \
  "${PERCEPTION_PACKAGE}/test/perception_safety_policy_test.cpp" \
  "${PERCEPTION_PACKAGE}/test/perception_health_test.cpp" \
  "${PERCEPTION_PACKAGE}/include/robot_perception/perception_health.hpp" \
  "${PERCEPTION_PACKAGE}/include/robot_perception/target_manager.hpp" \
  "${PERCEPTION_PACKAGE}/include/robot_perception/inspection_event_policy.hpp" \
  "${PERCEPTION_PACKAGE}/include/robot_perception/perception_safety_policy.hpp" \
  "${PERCEPTION_PACKAGE}/src/target_manager.cpp" \
  "${PERCEPTION_PACKAGE}/src/inspection_event_policy.cpp" \
  "${PERCEPTION_PACKAGE}/src/perception_safety_policy.cpp" \
  "${PERCEPTION_PACKAGE}/src/perception_health.cpp" \
  "${PERCEPTION_PACKAGE}/src/perception_diagnostics_node.cpp" \
  "${PERCEPTION_INTERFACES}/CMakeLists.txt" \
  "${PERCEPTION_INTERFACES}/package.xml" \
  "${PERCEPTION_INTERFACES}/msg/DetectedObject3D.msg" \
  "${PERCEPTION_INTERFACES}/msg/PerceptionEvent.msg" \
  "${PERCEPTION_INTERFACES}/msg/PerceptionSafetyEvent.msg" \
  "src/robot_tasks/config/visual_inspection.yaml" \
  "src/robot_tasks/config/visual_inspection_phase5_validation.yaml" \
  "${PERCEPTION_PACKAGE}/config/tracking_phase5_validation.yaml" \
  "src/robot_tasks/include/robot_tasks/observation_pose_planner.hpp" \
  "src/robot_tasks/include/robot_tasks/visual_inspection_mission.hpp" \
  "src/robot_tasks/src/observation_pose_planner.cpp" \
  "src/robot_tasks/src/visual_inspection_mission.cpp" \
  "src/robot_tasks/src/visual_inspection_task_node.cpp" \
  "src/robot_tasks/test/observation_pose_planner_test.cpp" \
  "src/robot_tasks/test/visual_inspection_mission_test.cpp" \
  "src/robot_simulation/models/person_standing/model.sdf" \
  "src/robot_simulation/models/person_standing/LICENSE" \
  "src/robot_simulation/models/person_standing/ATTRIBUTION.md" \
  "src/robot_simulation/models/person_standing/meshes/standing.dae" \
  "scripts/prepare_phase3_detector_model.sh" \
  "scripts/check_factory_patrol_detector_runtime.sh" \
  "scripts/check_factory_patrol_target_manager_runtime.sh" \
  "scripts/check_factory_patrol_visual_inspection_runtime.sh" \
  "scripts/check_factory_patrol_perception_safety_runtime.sh" \
  "scripts/check_factory_patrol_perception_diagnostics_runtime.sh" \
  "scripts/perception_safety_runtime_probe.py" \
  "scripts/perception_diagnostics_runtime_probe.py" \
  "scripts/run_factory_patrol_benchmarks.sh" \
  "${EXPERIMENT_PACKAGE}/config/factory_patrol_benchmark.yaml" \
  "${EXPERIMENT_PACKAGE}/robot_experiments/benchmark_metrics.py" \
  "${EXPERIMENT_PACKAGE}/scripts/factory_patrol_benchmark" \
  "${EXPERIMENT_PACKAGE}/test/test_benchmark_metrics.py" \
  "${SCENARIO_DOC}" "${ARCH_DOC}" "${README_FILE}" "${EXPERIMENT_DOC}"; do
  require_file "${file}"
done

require_grep '<static>true</static>' "src/robot_simulation/models/person_standing/model.sdf" "Phase 3 person target must be static"
if grep -q '<collision' "${ROOT_DIR}/src/robot_simulation/models/person_standing/model.sdf"; then
  fail "Phase 3 person target must remain visual-only"
fi
pass "Phase 3 person target is visual-only"

require_grep "factory_patrol" "${WORLD_FILE}" "factory patrol world name is missing"
require_grep "factory_patrol_main_road" "${WORLD_FILE}" "main patrol road visual is missing"
require_grep "factory_patrol_equipment_area" "${WORLD_FILE}" "equipment area visual is missing"
require_grep "narrow_corridor" "${WORLD_FILE}" "narrow corridor marker is missing"
require_grep "turning_area" "${WORLD_FILE}" "turning area visual is missing"
require_grep "station_A" "${WORLD_FILE}" "station_A visual marker is missing"
require_grep "station_B" "${WORLD_FILE}" "station_B visual marker is missing"
require_grep "station_C" "${WORLD_FILE}" "station_C visual marker is missing"
require_grep "factory_dock_pad" "${WORLD_FILE}" "dock visual marker is missing"
require_grep "slow_zone" "${WORLD_FILE}" "slow zone visual marker is missing"
require_grep "no_go_zone" "${WORLD_FILE}" "no-go planned visual marker is missing"
require_grep "showcase_floor_finish" "${WORLD_FILE}" "showcase floor finish detail is missing"
require_grep "showcase_static_station_signs" "${WORLD_FILE}" "showcase station signs are missing"
require_grep "showcase_large_factory_layout" "${WORLD_FILE}" "showcase large factory layout layer is missing"
require_grep "showcase_storage_industrial_detail" "${WORLD_FILE}" "showcase storage detail is missing"
require_grep "showcase_packing_workcell_detail" "${WORLD_FILE}" "showcase packing detail is missing"
require_grep "industrial_v2_factory_layout" "${INDUSTRIAL_WORLD_FILE}" "industrial V2 factory layout is missing"
require_grep "24 16 0.05" "${INDUSTRIAL_WORLD_FILE}" "industrial V2 floor size is not documented in SDF"
require_grep "GzSceneManager" "${INDUSTRIAL_WORLD_FILE}" "industrial V2 Scene Manager plugin is missing"
require_grep "CameraTracking" "${INDUSTRIAL_WORLD_FILE}" "industrial V2 Camera Tracking plugin is missing"
for world in "${WORLD_FILE}" "${INDUSTRIAL_WORLD_FILE}"; do
  require_grep 'sensor name="rgbd_camera" type="rgbd_camera"' "${world}" "RGB-D camera sensor is missing from ${world}"
  require_camera_sensor_grep '<pose>0.58 0 0.42 0 0 0</pose>' "${world}" "RGB-D camera extrinsic is inconsistent in ${world}"
  require_camera_sensor_grep '<topic>/camera</topic>' "${world}" "RGB-D Gazebo topic root is missing from ${world}"
  require_camera_sensor_grep '<gz_frame_id>camera_color_optical_frame</gz_frame_id>' "${world}" "RGB-D frame ID is missing from ${world}"
  require_camera_sensor_grep '<optical_frame_id>camera_color_optical_frame</optical_frame_id>' "${world}" "camera info optical frame is missing from ${world}"
  require_camera_sensor_grep '<width>640</width>' "${world}" "RGB-D image width is missing from ${world}"
  require_camera_sensor_grep '<height>480</height>' "${world}" "RGB-D image height is missing from ${world}"
  require_grep 'phase2_geometry_validation_target' "${world}" "Phase 2 geometry target is missing from ${world}"
  require_grep 'P_gt: \[2.70, 0.00, 0.495\]' "${world}" "Phase 2 ground truth is not documented in ${world}"
  require_grep 'phase3_person_detection_target' "${world}" "Phase 3 person target is missing from ${world}"
  require_grep 'model://person_standing' "${world}" "Phase 3 person asset URI is missing from ${world}"
  require_grep 'phase6_person_safety_target' "${world}" "Phase 6 close-range person fixture is missing from ${world}"
  require_grep '<scale>0.40 0.40 0.40</scale>' "${world}" "Phase 6 close-range person fixture scale is missing from ${world}"
done

require_grep 'name="camera_color_optical_frame"' "${ROBOT_XACRO}" "camera optical frame is missing from robot description"
require_grep 'rpy="-1.57079632679 0 -1.57079632679"' "${ROBOT_XACRO}" "camera optical-frame orientation is not conventional"
require_grep 'name="camera_x" value="0.58"' "${ROBOT_XACRO}" "authoritative camera X extrinsic is missing"
require_grep 'name="camera_y" value="0.0"' "${ROBOT_XACRO}" "authoritative camera Y extrinsic is missing"
require_grep 'name="camera_z" value="0.42"' "${ROBOT_XACRO}" "authoritative camera Z extrinsic is missing"

require_grep "station_A" "${STATIONS_FILE}" "station_A config is missing"
require_grep "station_B" "${STATIONS_FILE}" "station_B config is missing"
require_grep "station_C" "${STATIONS_FILE}" "station_C config is missing"
require_grep "dock" "${STATIONS_FILE}" "dock config is missing"
require_grep "factory_slow_corridor" "${ZONES_FILE}" "slow corridor zone is missing"
require_grep "keepout_planned" "${ZONES_FILE}" "planned keepout zone is missing"
require_grep "factory_patrol_loop" "${ROUTE_FILE}" "route name is missing"
require_grep "station_A" "${ROUTE_FILE}" "route does not reference station_A"
require_grep "station_B" "${ROUTE_FILE}" "route does not reference station_B"
require_grep "station_C" "${ROUTE_FILE}" "route does not reference station_C"

require_grep "world_file" "${SIM_LAUNCH}" "sim.launch.py does not expose world_file"
require_grep "world_name" "${SIM_LAUNCH}" "sim.launch.py does not expose world_name"
require_grep "/camera/color/image_raw" "${SIM_LAUNCH}" "RGB image bridge/remapping is missing"
require_grep "/camera/depth/image_raw" "${SIM_LAUNCH}" "depth image bridge/remapping is missing"
require_grep "/camera/color/camera_info" "${SIM_LAUNCH}" "camera info bridge/remapping is missing"
require_grep "GZ_SIM_RESOURCE_PATH" "${SIM_LAUNCH}" "Gazebo model resource path setup is missing"
require_grep "/camera/color/image_raw" "${SHOWCASE_RVIZ}" "Factory Patrol RViz RGB display is missing"
require_grep "/perception/markers" "${SHOWCASE_RVIZ}" "Factory Patrol RViz geometry marker is missing"
require_grep "/perception/debug_image" "${SHOWCASE_RVIZ}" "Factory Patrol RViz detector debug image is missing"
require_grep "Managed Targets" "${SHOWCASE_RVIZ}" "Factory Patrol RViz managed-target display is missing"
require_grep "/inspection/observation_pose" "${SHOWCASE_RVIZ}" "Factory Patrol RViz observation pose is missing"
require_grep "perception.launch.py" "${DEMO_LAUNCH}" "Factory Patrol launch does not include perception"
require_grep "geometry_input_mode" "${DEMO_LAUNCH}" "Factory Patrol launch does not expose geometry input mode"
require_grep "use_detector" "${DEMO_LAUNCH}" "Factory Patrol launch does not expose detector enablement"
require_grep "use_visual_inspection" "${DEMO_LAUNCH}" "Factory Patrol launch does not expose visual inspection"
require_grep "use_perception_safety" "${DEMO_LAUNCH}" "Factory Patrol launch does not expose perception safety"
require_grep "use_perception_diagnostics" "${DEMO_LAUNCH}" "Factory Patrol launch does not expose perception diagnostics"
require_grep "use_perception_system_monitor" "${DEMO_LAUNCH}" "Factory Patrol launch does not expose perception system monitoring"
require_grep "visual_inspection_task_node" "${DEMO_LAUNCH}" "Factory Patrol launch does not start the visual inspection task"
require_grep "navigate_sequence_server_node" "${DEMO_LAUNCH}" "Factory Patrol launch does not use the existing navigation adapter"
require_grep "robot_perception" "${DEMO_LAUNCH}" "Factory Patrol launch does not reference robot_perception"
require_grep "factory_patrol.sdf" "${DEMO_LAUNCH}" "factory patrol launch does not use factory world"
require_grep "world_file" "${DEMO_LAUNCH}" "factory patrol launch does not expose world override"
require_grep "factory_patrol_industrial.sdf" "${DEMO_LAUNCH}" "factory patrol launch does not mention industrial V2 world"
require_grep "factory_patrol_showcase.rviz" "${DEMO_LAUNCH}" "factory patrol launch does not use factory RViz showcase config"
require_grep "factory_patrol_demo.launch.py" "${RUN_SCRIPT}" "run script does not reference factory launch"

require_grep "Phase 5A" "${SCENARIO_DOC}" "simulation docs do not mention Phase 5A"
require_grep "factory_patrol.sdf" "${SCENARIO_DOC}" "simulation docs do not document factory world"
require_grep "factory_patrol_industrial.sdf" "${SCENARIO_DOC}" "simulation docs do not document industrial V2 world"
require_grep "factory_patrol_stations.yaml" "${SCENARIO_DOC}" "simulation docs do not document stations"
require_grep "factory_patrol_zones.yaml" "${SCENARIO_DOC}" "simulation docs do not document zones"
require_grep "factory_patrol_route.yaml" "${SCENARIO_DOC}" "simulation docs do not document route"
require_grep "camera_color_optical_frame" "${SCENARIO_DOC}" "simulation docs do not document the camera optical frame"
require_grep "/camera/depth/image_raw" "${SCENARIO_DOC}" "simulation docs do not document the depth topic"
require_grep "/perception/geometry/map_point" "${SCENARIO_DOC}" "simulation docs do not document the Phase 2 map point"
require_grep "ApproximateTime" "${SCENARIO_DOC}" "simulation docs do not document Phase 2 synchronization"
require_grep "/perception/detections_2d" "${SCENARIO_DOC}" "simulation docs do not document Phase 3 detections"
require_grep "OpenCV Zoo" "${SCENARIO_DOC}" "simulation docs do not document detector model setup"
require_grep "/perception/objects_3d" "${SCENARIO_DOC}" "simulation docs do not document Phase 4 managed targets"
require_grep "TENTATIVE" "${SCENARIO_DOC}" "simulation docs do not document target lifecycle"
require_grep "INSPECTION_REQUIRED" "${SCENARIO_DOC}" "simulation docs do not document Phase 5 events"
require_grep "standoff_distance" "${SCENARIO_DOC}" "simulation docs do not document Phase 5 standoff"
require_grep "/perception/safety_event" "${SCENARIO_DOC}" "simulation docs do not document the Phase 6 safety event"
require_grep "/perception/diagnostics" "${SCENARIO_DOC}" "simulation docs do not document Phase 7 diagnostics"
require_grep "factory_patrol" "${ARCH_DOC}" "architecture docs do not mention factory patrol"
require_grep "Factory Patrol" "${README_FILE}" "README does not mention Factory Patrol"
require_grep "check_factory_patrol_assets.sh" "${README_FILE}" "README does not mention factory check script"
require_grep "run_factory_patrol_benchmarks.sh" "${EXPERIMENT_DOC}" "experiment docs do not document the Phase 8 runner"
require_grep "nearest rank" "${EXPERIMENT_DOC}" "experiment docs do not define benchmark percentiles"

require_grep "class DepthProjector" "${PERCEPTION_PACKAGE}/include/robot_perception/depth_projector.hpp" "DepthProjector class is missing"
require_grep "32FC1" "${PERCEPTION_PACKAGE}/src/depth_projector.cpp" "32FC1 depth support is missing"
require_grep "lookupTransform" "${PERCEPTION_PACKAGE}/src/geometry_validation_node.cpp" "observation-time TF lookup is missing"
require_grep "ApproximateTime" "${PERCEPTION_PACKAGE}/src/geometry_validation_node.cpp" "camera stream synchronization is missing"
require_grep "class DetectorBackend" "${PERCEPTION_PACKAGE}/robot_perception/detector_backend.py" "replaceable detector backend is missing"
require_grep "vision_msgs" "${PERCEPTION_PACKAGE}/package.xml" "standard 2D detection interface dependency is missing"
require_grep "geometry_input_mode" "${PERCEPTION_PACKAGE}/src/geometry_validation_node.cpp" "synthetic/detector geometry selection is missing"
require_grep "diagnostic_msgs::msg::DiagnosticArray" "${PERCEPTION_PACKAGE}/src/perception_diagnostics_node.cpp" "standard perception diagnostics output is missing"
require_grep "perception/pipeline" "${PERCEPTION_PACKAGE}/src/perception_diagnostics_node.cpp" "perception pipeline health aggregation is missing"
require_grep '"device",' "${PERCEPTION_PACKAGE}/robot_perception/detector_node.py" "detector diagnostics do not report the actual device"
require_grep "warmup_samples" "${EXPERIMENT_PACKAGE}/config/factory_patrol_benchmark.yaml" "Phase 8 detector warmup policy is missing"
require_grep "percentile_nearest_rank" "${EXPERIMENT_PACKAGE}/robot_experiments/benchmark_metrics.py" "Phase 8 statistics helper is missing"
require_grep "invalid_depth" "${EXPERIMENT_PACKAGE}/scripts/factory_patrol_benchmark" "Phase 8 invalid-depth profile is missing"
require_grep "monitor_perception" "src/robot_utils/src/system_monitor_node.cpp" "system monitor perception integration is missing"
require_grep "class TargetManager" "${PERCEPTION_PACKAGE}/include/robot_perception/target_manager.hpp" "TargetManager class is missing"
require_grep "max_match_distance" "${PERCEPTION_PACKAGE}/config/tracking.yaml" "TargetManager association configuration is missing"
require_grep "ema_alpha" "${PERCEPTION_PACKAGE}/config/tracking.yaml" "TargetManager EMA configuration is missing"
require_grep "lost_retirement_frames" "${PERCEPTION_PACKAGE}/config/tracking.yaml" "TargetManager retirement configuration is missing"
require_grep "robot_interfaces_perception" "${PERCEPTION_PACKAGE}/package.xml" "managed target interface dependency is missing"
require_grep "DetectedObject3D.msg" "${PERCEPTION_INTERFACES}/CMakeLists.txt" "managed target interface is not generated"
require_grep "uint32 target_id" "${PERCEPTION_INTERFACES}/msg/DetectedObject3D.msg" "managed target ID field is missing"
require_grep "uint8 tracking_state" "${PERCEPTION_INTERFACES}/msg/DetectedObject3D.msg" "managed lifecycle field is missing"
require_grep "PerceptionEvent.msg" "${PERCEPTION_INTERFACES}/CMakeLists.txt" "perception event interface is not generated"
require_grep "string event_type" "${PERCEPTION_INTERFACES}/msg/PerceptionEvent.msg" "perception event type field is missing"
require_grep "PerceptionSafetyEvent.msg" "${PERCEPTION_INTERFACES}/CMakeLists.txt" "perception safety interface is not generated"
require_grep "uint8 safety_state" "${PERCEPTION_INTERFACES}/msg/PerceptionSafetyEvent.msg" "perception safety state field is missing"
require_grep "/perception/objects_3d" "${PERCEPTION_PACKAGE}/config/tracking.yaml" "managed target topic is missing"
require_grep "/perception/events" "${PERCEPTION_PACKAGE}/config/tracking.yaml" "Phase 5 perception event topic is missing"
require_grep "inspection.allowed_classes: \[person\]" "${PERCEPTION_PACKAGE}/config/tracking_phase5_validation.yaml" "Phase 5 validation perception allowlist is not explicit"
require_grep "inspection.allowed_classes: \[person\]" "src/robot_tasks/config/visual_inspection_phase5_validation.yaml" "Phase 5 validation task allowlist is not explicit"
require_grep "MarkProcessed" "${PERCEPTION_PACKAGE}/src/geometry_validation_node.cpp" "Phase 5 completion feedback is not connected to TargetManager"
require_grep "INSPECTION_REQUIRED" "src/robot_tasks/src/visual_inspection_task_node.cpp" "visual inspection task does not consume Phase 5 events"
require_grep "/navigate_sequence" "src/robot_tasks/config/visual_inspection.yaml" "visual inspection task does not use the existing navigation adapter"
require_grep "class PerceptionSafetyPolicy" "${PERCEPTION_PACKAGE}/include/robot_perception/perception_safety_policy.hpp" "Phase 6 perception safety policy is missing"
require_grep "danger_zone" "${PERCEPTION_PACKAGE}/config/safety_zones.yaml" "Phase 6 danger zone configuration is missing"
require_grep "/perception/safety_event" "${PERCEPTION_PACKAGE}/config/safety.yaml" "Phase 6 safety topic configuration is missing"
require_grep "CONFIRMED" "${SCENARIO_DOC}" "Phase 6 eligible TargetManager state is not documented"
if rg -n '/(nav2_)?cmd_vel|create_publisher<geometry_msgs::msg::Twist>' "${ROOT_DIR}/${PERCEPTION_PACKAGE}" >/dev/null; then
  fail "robot_perception contains an out-of-scope velocity publisher or topic"
fi
pass "robot_perception emits semantic mission/safety events without publishing velocity"

pass "factory patrol world, config assets, map note, demo entry, scripts, and docs are present"
