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

WORLD_FILE="src/robot_simulation/worlds/factory_patrol.sdf"
INDUSTRIAL_WORLD_FILE="src/robot_simulation/worlds/factory_patrol_industrial.sdf"
STATIONS_FILE="src/robot_simulation/config/factory_patrol_stations.yaml"
ZONES_FILE="src/robot_simulation/config/factory_patrol_zones.yaml"
ROUTE_FILE="src/robot_simulation/config/factory_patrol_route.yaml"
MAP_README="src/robot_navigation/maps/factory_patrol_map_README.md"
DEMO_LAUNCH="src/robot_bringup/launch/factory_patrol_demo.launch.py"
RUN_SCRIPT="scripts/run_factory_patrol_demo.sh"
SIM_LAUNCH="src/robot_simulation/launch/sim.launch.py"
SCENARIO_DOC="docs/simulation_scenarios.md"
ARCH_DOC="docs/architecture.md"
README_FILE="README.md"

for file in "${WORLD_FILE}" "${INDUSTRIAL_WORLD_FILE}" "${STATIONS_FILE}" "${ZONES_FILE}" "${ROUTE_FILE}" \
  "${MAP_README}" "${DEMO_LAUNCH}" "${RUN_SCRIPT}" "${SIM_LAUNCH}" \
  "${SCENARIO_DOC}" "${ARCH_DOC}" "${README_FILE}"; do
  require_file "${file}"
done

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
require_grep "factory_patrol" "${ARCH_DOC}" "architecture docs do not mention factory patrol"
require_grep "Factory Patrol" "${README_FILE}" "README does not mention Factory Patrol"
require_grep "check_factory_patrol_assets.sh" "${README_FILE}" "README does not mention factory check script"

pass "factory patrol world, config assets, map note, demo entry, scripts, and docs are present"
