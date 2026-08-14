#include "robot_perception/inspection_event_policy.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace robot_perception {

InspectionEventPolicy::InspectionEventPolicy(InspectionEventPolicyConfig config)
    : config_(std::move(config)) {
  if (!std::isfinite(config_.min_confidence) || config_.min_confidence < 0.0 ||
      config_.min_confidence > 1.0) {
    throw std::invalid_argument("inspection min_confidence must be in [0, 1]");
  }
  config_.allowed_classes.erase(
      std::remove_if(
          config_.allowed_classes.begin(), config_.allowed_classes.end(),
          [](const std::string& class_name) { return class_name.empty(); }),
      config_.allowed_classes.end());
}

bool InspectionEventPolicy::ShouldEmit(const ManagedTarget& target) {
  if (target.state == TrackingState::kTentative &&
      emitted_target_ids_.count(target.target_id) != 0U) {
    emitted_target_ids_.erase(target.target_id);
  }
  if (!config_.enabled || target.target_id == 0U ||
      target.state != TrackingState::kConfirmed || !target.depth_valid ||
      !std::isfinite(target.confidence) ||
      target.confidence < config_.min_confidence ||
      emitted_target_ids_.count(target.target_id) != 0U) {
    return false;
  }
  const bool class_allowed =
      std::find(config_.allowed_classes.begin(), config_.allowed_classes.end(),
                target.class_name) != config_.allowed_classes.end();
  if (!class_allowed) {
    return false;
  }
  emitted_target_ids_.insert(target.target_id);
  return true;
}

bool InspectionEventPolicy::HasEmitted(const std::uint32_t target_id) const {
  return emitted_target_ids_.count(target_id) != 0U;
}

}  // namespace robot_perception
