#include "robot_tasks/observation_pose_planner.hpp"

#include <cmath>

namespace robot_tasks {
namespace {

constexpr double kMinimumDirectionNorm = 1.0e-6;
constexpr double kMinimumQuaternionNorm = 1.0e-9;

bool FiniteQuaternion(const geometry_msgs::msg::Quaternion& quaternion) {
  return std::isfinite(quaternion.x) && std::isfinite(quaternion.y) &&
         std::isfinite(quaternion.z) && std::isfinite(quaternion.w) &&
         std::hypot(std::hypot(quaternion.x, quaternion.y),
                    std::hypot(quaternion.z, quaternion.w)) >
             kMinimumQuaternionNorm;
}

double YawFromQuaternion(const geometry_msgs::msg::Quaternion& quaternion) {
  const double norm = std::hypot(
      std::hypot(quaternion.x, quaternion.y),
      std::hypot(quaternion.z, quaternion.w));
  const double x = quaternion.x / norm;
  const double y = quaternion.y / norm;
  const double z = quaternion.z / norm;
  const double w = quaternion.w / norm;
  return std::atan2(2.0 * (w * z + x * y),
                    1.0 - 2.0 * (y * y + z * z));
}

}  // namespace

bool IsFiniteValidPose(const geometry_msgs::msg::PoseStamped& pose) {
  return !pose.header.frame_id.empty() &&
         std::isfinite(pose.pose.position.x) &&
         std::isfinite(pose.pose.position.y) &&
         std::isfinite(pose.pose.position.z) &&
         FiniteQuaternion(pose.pose.orientation);
}

ObservationPoseResult ComputeObservationPose(
    const geometry_msgs::msg::PoseStamped& target_pose,
    const geometry_msgs::msg::PoseStamped& robot_pose,
    const double standoff_distance) {
  if (target_pose.header.frame_id != "map" ||
      robot_pose.header.frame_id != "map") {
    return {std::nullopt, "target and robot poses must use the map frame"};
  }
  if (!IsFiniteValidPose(target_pose) || !IsFiniteValidPose(robot_pose)) {
    return {std::nullopt, "target and robot poses must be finite with valid quaternions"};
  }
  if (!std::isfinite(standoff_distance) || standoff_distance <= 0.0) {
    return {std::nullopt, "standoff_distance must be positive and finite"};
  }

  double direction_x = robot_pose.pose.position.x - target_pose.pose.position.x;
  double direction_y = robot_pose.pose.position.y - target_pose.pose.position.y;
  const double direction_norm = std::hypot(direction_x, direction_y);
  if (direction_norm < kMinimumDirectionNorm) {
    const double robot_yaw = YawFromQuaternion(robot_pose.pose.orientation);
    direction_x = -std::cos(robot_yaw);
    direction_y = -std::sin(robot_yaw);
  } else {
    direction_x /= direction_norm;
    direction_y /= direction_norm;
  }

  geometry_msgs::msg::PoseStamped observation;
  observation.header = target_pose.header;
  observation.pose.position.x =
      target_pose.pose.position.x + direction_x * standoff_distance;
  observation.pose.position.y =
      target_pose.pose.position.y + direction_y * standoff_distance;
  observation.pose.position.z = robot_pose.pose.position.z;
  const double yaw = std::atan2(
      target_pose.pose.position.y - observation.pose.position.y,
      target_pose.pose.position.x - observation.pose.position.x);
  observation.pose.orientation.z = std::sin(yaw * 0.5);
  observation.pose.orientation.w = std::cos(yaw * 0.5);

  if (!IsFiniteValidPose(observation)) {
    return {std::nullopt, "computed observation pose is invalid"};
  }
  return {observation, ""};
}

}  // namespace robot_tasks
