#pragma once

#include <optional>
#include <string>
#include <vector>

#include "robot_tasks/mission_preflight.hpp"

namespace robot_tasks {

struct StationTransportSubmitGateRequest {
  std::string mission_id;
  bool mission_already_active_or_queued = false;
  bool station_catalog_loaded = true;
  MissionPreflightResult preflight;
  std::optional<std::vector<std::string>> route_path;
  bool traffic_intersection_locks_enabled = false;
};

struct StationTransportSubmitGateDecision {
  bool accepted = false;
  std::string mission_id;
  std::string message;
  std::vector<std::string> route_lock_ids;
};

StationTransportSubmitGateDecision PlanStationTransportSubmitGate(
    const StationTransportSubmitGateRequest& request);

}  // namespace robot_tasks
