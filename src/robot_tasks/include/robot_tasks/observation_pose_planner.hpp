#pragma once

#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace robot_tasks {

struct ObservationPoseResult {
  std::optional<geometry_msgs::msg::PoseStamped> pose;
  std::string error;
};

ObservationPoseResult ComputeObservationPose(
    const geometry_msgs::msg::PoseStamped& target_pose,
    const geometry_msgs::msg::PoseStamped& robot_pose,
    double standoff_distance);

bool IsFiniteValidPose(const geometry_msgs::msg::PoseStamped& pose);

}  // namespace robot_tasks
