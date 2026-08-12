#pragma once

#include <optional>

#include "robot_perception/perception_types.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace robot_perception {

class DepthProjector {
 public:
  explicit DepthProjector(DepthSamplingConfig config = {});

  ProjectionResult Project(
      const BoundingBox2D& bbox,
      const sensor_msgs::msg::Image& depth_image,
      const sensor_msgs::msg::CameraInfo& camera_info) const;

  static std::optional<CameraIntrinsics> ParseIntrinsics(
      const sensor_msgs::msg::CameraInfo& camera_info);

 private:
  ProjectionResult SampleDepth(
      const BoundingBox2D& bbox,
      const sensor_msgs::msg::Image& depth_image) const;

  DepthSamplingConfig config_;
  bool config_valid_ = false;
};

}  // namespace robot_perception
