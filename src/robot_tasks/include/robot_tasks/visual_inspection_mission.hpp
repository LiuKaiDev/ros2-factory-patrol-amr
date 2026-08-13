#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "robot_interfaces_perception/msg/perception_event.hpp"

namespace robot_tasks {

enum class VisualInspectionState {
  kIdle,
  kRequested,
  kNavigating,
  kSucceeded,
  kFailed,
};

struct VisualInspectionConfig {
  bool enabled = true;
  std::vector<std::string> allowed_classes{"chair"};
  double min_confidence = 0.5;
  double standoff_distance = 1.2;
  bool processed_on_success = true;
};

struct VisualInspectionDecision {
  bool accepted = false;
  std::string reason;
  std::optional<geometry_msgs::msg::PoseStamped> observation_pose;
};

struct VisualInspectionOutcome {
  bool publish_completion = false;
  std::uint32_t target_id = 0U;
};

class VisualInspectionMission {
 public:
  explicit VisualInspectionMission(VisualInspectionConfig config = {});

  VisualInspectionDecision Request(
      const robot_interfaces_perception::msg::PerceptionEvent& event,
      const geometry_msgs::msg::PoseStamped& robot_pose);
  void MarkNavigating();
  VisualInspectionOutcome Finish(bool success);

  VisualInspectionState state() const { return state_; }
  std::uint32_t active_target_id() const { return active_target_id_; }
  bool HasSeen(std::uint32_t target_id) const;

 private:
  VisualInspectionConfig config_;
  VisualInspectionState state_ = VisualInspectionState::kIdle;
  std::uint32_t active_target_id_ = 0U;
  std::unordered_map<std::uint32_t, std::int64_t> seen_event_stamps_ns_;
};

const char* VisualInspectionStateName(VisualInspectionState state);

}  // namespace robot_tasks
