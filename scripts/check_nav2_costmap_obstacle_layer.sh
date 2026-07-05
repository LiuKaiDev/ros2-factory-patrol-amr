#!/usr/bin/env bash
set -euo pipefail

CONFIG_FILE="${1:-src/robot_navigation/config/nav2_basic.yaml}"

fail() {
  echo "FAIL: $*" >&2
  exit 1
}

pass() {
  echo "PASS: $*"
}

[[ -f "${CONFIG_FILE}" ]] || fail "basic Nav2 config not found: ${CONFIG_FILE}"

grep -q "local_costmap:" "${CONFIG_FILE}" ||
  fail "local_costmap section is missing"
grep -q "obstacle_layer" "${CONFIG_FILE}" ||
  fail "local_costmap obstacle_layer is missing"
grep -q "nav2_costmap_2d::ObstacleLayer" "${CONFIG_FILE}" ||
  fail "ObstacleLayer plugin is missing"
grep -q "observation_sources: scan" "${CONFIG_FILE}" ||
  fail "LaserScan observation source is missing"
grep -q "topic: /scan" "${CONFIG_FILE}" ||
  fail "LaserScan topic /scan is missing"
grep -q "footprint:" "${CONFIG_FILE}" ||
  fail "robot footprint is missing"
grep -q "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController" "${CONFIG_FILE}" ||
  fail "RPP controller plugin is missing"
grep -q "use_collision_detection: true" "${CONFIG_FILE}" ||
  fail "RPP collision detection is missing"

pass "Nav2 basic costmap obstacle layer and RPP collision checks are configured in ${CONFIG_FILE}"
