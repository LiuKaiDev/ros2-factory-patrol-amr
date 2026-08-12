#pragma once

#include <cstddef>

#include "geometry_msgs/msg/point.hpp"

namespace robot_perception {

struct BoundingBox2D {
  double center_u = 0.0;
  double center_v = 0.0;
  double width = 0.0;
  double height = 0.0;
};

struct CameraIntrinsics {
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  std::size_t image_width = 0;
  std::size_t image_height = 0;
};

struct DepthSamplingConfig {
  double min_depth = 0.2;
  double max_depth = 8.0;
  double roi_ratio = 0.3;
  std::size_t min_valid_samples = 3;
};

enum class ProjectionStatus {
  kValid,
  kInvalidConfiguration,
  kInvalidIntrinsics,
  kInvalidBoundingBox,
  kInvalidDepthImage,
  kUnsupportedDepthEncoding,
  kInsufficientValidDepth,
};

struct ProjectionResult {
  ProjectionStatus status = ProjectionStatus::kInvalidDepthImage;
  geometry_msgs::msg::Point point;
  double depth = 0.0;
  std::size_t valid_sample_count = 0;

  bool valid() const { return status == ProjectionStatus::kValid; }
};

const char* ProjectionStatusName(ProjectionStatus status);

}  // namespace robot_perception
