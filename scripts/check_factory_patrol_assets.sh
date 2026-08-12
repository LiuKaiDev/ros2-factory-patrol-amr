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
SCENARIO_DOC="docs/simulation_scenarios.md"
ARCH_DOC="docs/architecture.md"
README_FILE="README.md"

for file in "${WORLD_FILE}" "${INDUSTRIAL_WORLD_FILE}" "${STATIONS_FILE}" "${ZONES_FILE}" "${ROUTE_FILE}" \
  "${MAP_README}" "${DEMO_LAUNCH}" "${RUN_SCRIPT}" "${SIM_LAUNCH}" \
  "${ROBOT_XACRO}" "${SHOWCASE_RVIZ}" "${PERCEPTION_PACKAGE}/CMakeLists.txt" \
  "${PERCEPTION_PACKAGE}/package.xml" "${PERCEPTION_PACKAGE}/config/depth.yaml" \
  "${PERCEPTION_PACKAGE}/config/detector.yaml" \
  "${PERCEPTION_PACKAGE}/launch/geometry_validation.launch.py" \
  "${PERCEPTION_PACKAGE}/launch/perception.launch.py" \
  "${PERCEPTION_PACKAGE}/robot_perception/detector_backend.py" \
  "${PERCEPTION_PACKAGE}/robot_perception/detector_node.py" \
  "${PERCEPTION_PACKAGE}/test/test_detection_utils.py" \
  "${PERCEPTION_PACKAGE}/test/depth_projector_test.cpp" \
  "src/robot_simulation/models/person_standing/model.sdf" \
  "src/robot_simulation/models/person_standing/LICENSE" \
  "src/robot_simulation/models/person_standing/ATTRIBUTION.md" \
  "src/robot_simulation/models/person_standing/meshes/standing.dae" \
  "scripts/prepare_phase3_detector_model.sh" \
  "scripts/check_factory_patrol_detector_runtime.sh" \
  "${SCENARIO_DOC}" "${ARCH_DOC}" "${README_FILE}"; do
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
require_grep "perception.launch.py" "${DEMO_LAUNCH}" "Factory Patrol launch does not include perception"
require_grep "geometry_input_mode" "${DEMO_LAUNCH}" "Factory Patrol launch does not expose geometry input mode"
require_grep "use_detector" "${DEMO_LAUNCH}" "Factory Patrol launch does not expose detector enablement"
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
require_grep "factory_patrol" "${ARCH_DOC}" "architecture docs do not mention factory patrol"
require_grep "Factory Patrol" "${README_FILE}" "README does not mention Factory Patrol"
require_grep "check_factory_patrol_assets.sh" "${README_FILE}" "README does not mention factory check script"

require_grep "class DepthProjector" "${PERCEPTION_PACKAGE}/include/robot_perception/depth_projector.hpp" "DepthProjector class is missing"
require_grep "32FC1" "${PERCEPTION_PACKAGE}/src/depth_projector.cpp" "32FC1 depth support is missing"
require_grep "lookupTransform" "${PERCEPTION_PACKAGE}/src/geometry_validation_node.cpp" "observation-time TF lookup is missing"
require_grep "ApproximateTime" "${PERCEPTION_PACKAGE}/src/geometry_validation_node.cpp" "camera stream synchronization is missing"
require_grep "class DetectorBackend" "${PERCEPTION_PACKAGE}/robot_perception/detector_backend.py" "replaceable detector backend is missing"
require_grep "vision_msgs" "${PERCEPTION_PACKAGE}/package.xml" "standard 2D detection interface dependency is missing"
require_grep "geometry_input_mode" "${PERCEPTION_PACKAGE}/src/geometry_validation_node.cpp" "synthetic/detector geometry selection is missing"
if rg -n 'cmd_vel|TargetManager|target_id|objects_3d|perception/events|safety_event' "${ROOT_DIR}/${PERCEPTION_PACKAGE}" >/dev/null; then
  fail "robot_perception contains out-of-scope control or Phase 4+ implementation"
fi
pass "robot_perception stays within the Phase 3 detector and geometry scope"

pass "factory patrol world, config assets, map note, demo entry, scripts, and docs are present"
