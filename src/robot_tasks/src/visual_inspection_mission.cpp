#include "robot_tasks/visual_inspection_mission.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "robot_tasks/observation_pose_planner.hpp"

namespace robot_tasks {

const char* VisualInspectionStateName(const VisualInspectionState state) {
  switch (state) {
    case VisualInspectionState::kIdle:
      return "IDLE";
    case VisualInspectionState::kRequested:
      return "REQUESTED";
    case VisualInspectionState::kNavigating:
      return "NAVIGATING";
    case VisualInspectionState::kSucceeded:
      return "SUCCEEDED";
    case VisualInspectionState::kFailed:
      return "FAILED";
  }
  return "UNKNOWN";
}

VisualInspectionMission::VisualInspectionMission(VisualInspectionConfig config)
    : config_(std::move(config)) {
  if (!std::isfinite(config_.min_confidence) || config_.min_confidence < 0.0 ||
      config_.min_confidence > 1.0) {
    throw std::invalid_argument("inspection min_confidence must be in [0, 1]");
  }
  if (!std::isfinite(config_.standoff_distance) ||
      config_.standoff_distance <= 0.0) {
    throw std::invalid_argument("inspection standoff_distance must be positive and finite");
  }
}

VisualInspectionDecision VisualInspectionMission::Request(
    const robot_interfaces_perception::msg::PerceptionEvent& event,
    const geometry_msgs::msg::PoseStamped& robot_pose) {
  if (!config_.enabled) {
    return {false, "visual inspection is disabled", std::nullopt};
  }
  if (event.event_type !=
      robot_interfaces_perception::msg::PerceptionEvent::INSPECTION_REQUIRED) {
    return {false, "event is not INSPECTION_REQUIRED", std::nullopt};
  }
  const std::int64_t event_stamp_ns =
      static_cast<std::int64_t>(event.header.stamp.sec) * 1000000000LL +
      static_cast<std::int64_t>(event.header.stamp.nanosec);
  const auto prior_event = seen_event_stamps_ns_.find(event.target_id);
  if (event.target_id == 0U || event_stamp_ns < 0 ||
      (prior_event != seen_event_stamps_ns_.end() &&
       event_stamp_ns <= prior_event->second)) {
    return {false, "target event is invalid, stale, or duplicated", std::nullopt};
  }
  if (state_ == VisualInspectionState::kRequested ||
      state_ == VisualInspectionState::kNavigating) {
    return {false, "another visual inspection mission is active", std::nullopt};
  }
  if (std::find(config_.allowed_classes.begin(), config_.allowed_classes.end(),
                event.class_name) == config_.allowed_classes.end()) {
    return {false, "target class is not allowlisted", std::nullopt};
  }
  if (!std::isfinite(event.confidence) ||
      event.confidence < config_.min_confidence || event.confidence > 1.0F) {
    return {false, "target confidence is below threshold or invalid", std::nullopt};
  }
  const auto plan = ComputeObservationPose(
      event.target_pose, robot_pose, config_.standoff_distance);
  if (!plan.pose) {
    return {false, plan.error, std::nullopt};
  }

  active_target_id_ = event.target_id;
  seen_event_stamps_ns_[event.target_id] = event_stamp_ns;
  state_ = VisualInspectionState::kRequested;
  return {true, "inspection mission requested", plan.pose};
}

void VisualInspectionMission::MarkNavigating() {
  if (state_ == VisualInspectionState::kRequested) {
    state_ = VisualInspectionState::kNavigating;
  }
}

VisualInspectionOutcome VisualInspectionMission::Finish(const bool success) {
  if (state_ != VisualInspectionState::kRequested &&
      state_ != VisualInspectionState::kNavigating) {
    return {};
  }
  const std::uint32_t completed_target_id = active_target_id_;
  state_ = success ? VisualInspectionState::kSucceeded
                   : VisualInspectionState::kFailed;
  active_target_id_ = 0U;
  return {success && config_.processed_on_success, completed_target_id};
}

bool VisualInspectionMission::HasSeen(const std::uint32_t target_id) const {
  return seen_event_stamps_ns_.count(target_id) != 0U;
}

}  // namespace robot_tasks
