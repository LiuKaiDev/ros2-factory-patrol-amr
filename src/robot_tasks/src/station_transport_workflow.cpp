#include "robot_tasks/station_transport_workflow.hpp"

#include "robot_tasks/traffic_reservation.hpp"

namespace robot_tasks {

StationTransportSubmitGateDecision PlanStationTransportSubmitGate(
    const StationTransportSubmitGateRequest& request) {
  StationTransportSubmitGateDecision decision;
  decision.mission_id = request.mission_id;

  if (request.mission_already_active_or_queued) {
    decision.message =
        "station transport order already active or queued: " + request.mission_id;
    return decision;
  }
  if (!request.station_catalog_loaded) {
    decision.message = "station transport preflight rejected: failed to load station catalog";
    return decision;
  }
  if (!request.preflight.allowed) {
    decision.message = "station transport preflight rejected: " + request.preflight.message;
    return decision;
  }
  if (!request.route_path.has_value()) {
    decision.message = "station transport preflight rejected: no enabled station route";
    return decision;
  }

  decision.accepted = true;
  decision.route_lock_ids =
      BuildRouteLockIds(*request.route_path, request.traffic_intersection_locks_enabled);
  return decision;
}

}  // namespace robot_tasks
