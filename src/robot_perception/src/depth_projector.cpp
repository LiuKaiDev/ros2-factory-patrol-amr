#include "robot_perception/depth_projector.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

namespace robot_perception {
namespace {

bool IsFinite(const double value) { return std::isfinite(value); }

bool IsValidBoundingBox(const BoundingBox2D& bbox, const std::size_t width,
                        const std::size_t height) {
  return IsFinite(bbox.center_u) && IsFinite(bbox.center_v) && IsFinite(bbox.width) &&
         IsFinite(bbox.height) && bbox.width > 0.0 && bbox.height > 0.0 &&
         bbox.center_u >= 0.0 && bbox.center_v >= 0.0 &&
         bbox.center_u < static_cast<double>(width) &&
         bbox.center_v < static_cast<double>(height);
}

std::uint32_t ByteSwap32(const std::uint32_t value) {
  return ((value & 0x000000FFU) << 24U) | ((value & 0x0000FF00U) << 8U) |
         ((value & 0x00FF0000U) >> 8U) | ((value & 0xFF000000U) >> 24U);
}

bool HostIsBigEndian() {
  const std::uint16_t value = 0x0102U;
  return *reinterpret_cast<const std::uint8_t*>(&value) == 0x01U;
}

float ReadFloat32(const std::uint8_t* data, const bool message_is_big_endian) {
  std::uint32_t bits = 0U;
  std::memcpy(&bits, data, sizeof(bits));
  if (message_is_big_endian != HostIsBigEndian()) {
    bits = ByteSwap32(bits);
  }
  float value = 0.0F;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

double Median(std::vector<float>* values) {
  auto& samples = *values;
  const std::size_t middle = samples.size() / 2U;
  std::nth_element(samples.begin(), samples.begin() + middle, samples.end());
  const double upper = samples[middle];
  if (samples.size() % 2U != 0U) {
    return upper;
  }
  const auto lower_it = std::max_element(samples.begin(), samples.begin() + middle);
  return (static_cast<double>(*lower_it) + upper) * 0.5;
}

}  // namespace

const char* ProjectionStatusName(const ProjectionStatus status) {
  switch (status) {
    case ProjectionStatus::kValid:
      return "valid";
    case ProjectionStatus::kInvalidConfiguration:
      return "invalid_configuration";
    case ProjectionStatus::kInvalidIntrinsics:
      return "invalid_intrinsics";
    case ProjectionStatus::kInvalidBoundingBox:
      return "invalid_bounding_box";
    case ProjectionStatus::kInvalidDepthImage:
      return "invalid_depth_image";
    case ProjectionStatus::kUnsupportedDepthEncoding:
      return "unsupported_depth_encoding";
    case ProjectionStatus::kInsufficientValidDepth:
      return "insufficient_valid_depth";
  }
  return "unknown";
}

DepthProjector::DepthProjector(DepthSamplingConfig config) : config_(config) {
  config_valid_ = IsFinite(config_.min_depth) && IsFinite(config_.max_depth) &&
                  IsFinite(config_.roi_ratio) && config_.min_depth > 0.0 &&
                  config_.max_depth > config_.min_depth && config_.roi_ratio > 0.0 &&
                  config_.roi_ratio <= 1.0 && config_.min_valid_samples > 0U;
}

std::optional<CameraIntrinsics> DepthProjector::ParseIntrinsics(
    const sensor_msgs::msg::CameraInfo& camera_info) {
  CameraIntrinsics intrinsics;
  intrinsics.fx = camera_info.k[0];
  intrinsics.fy = camera_info.k[4];
  intrinsics.cx = camera_info.k[2];
  intrinsics.cy = camera_info.k[5];
  intrinsics.image_width = camera_info.width;
  intrinsics.image_height = camera_info.height;

  const bool valid = camera_info.width > 0U && camera_info.height > 0U &&
                     IsFinite(intrinsics.fx) && IsFinite(intrinsics.fy) &&
                     IsFinite(intrinsics.cx) && IsFinite(intrinsics.cy) &&
                     intrinsics.fx > 0.0 && intrinsics.fy > 0.0 && intrinsics.cx >= 0.0 &&
                     intrinsics.cy >= 0.0 &&
                     intrinsics.cx < static_cast<double>(camera_info.width) &&
                     intrinsics.cy < static_cast<double>(camera_info.height);
  if (!valid) {
    return std::nullopt;
  }
  return intrinsics;
}

ProjectionResult DepthProjector::SampleDepth(
    const BoundingBox2D& bbox, const sensor_msgs::msg::Image& depth_image) const {
  ProjectionResult result;
  if (!config_valid_) {
    result.status = ProjectionStatus::kInvalidConfiguration;
    return result;
  }
  if (depth_image.encoding != "32FC1") {
    result.status = ProjectionStatus::kUnsupportedDepthEncoding;
    return result;
  }
  constexpr std::size_t bytes_per_pixel = sizeof(float);
  const std::size_t width = depth_image.width;
  const std::size_t height = depth_image.height;
  const std::size_t minimum_step = width * bytes_per_pixel;
  if (width == 0U || height == 0U || depth_image.step < minimum_step ||
      depth_image.data.size() < static_cast<std::size_t>(depth_image.step) * height) {
    result.status = ProjectionStatus::kInvalidDepthImage;
    return result;
  }
  if (!IsValidBoundingBox(bbox, width, height)) {
    result.status = ProjectionStatus::kInvalidBoundingBox;
    return result;
  }

  const double roi_width = std::max(1.0, bbox.width * config_.roi_ratio);
  const double roi_height = std::max(1.0, bbox.height * config_.roi_ratio);
  const int min_u = std::max(0, static_cast<int>(std::ceil(bbox.center_u - roi_width * 0.5)));
  const int max_u = std::min(
      static_cast<int>(width) - 1,
      static_cast<int>(std::floor(bbox.center_u + roi_width * 0.5)));
  const int min_v = std::max(0, static_cast<int>(std::ceil(bbox.center_v - roi_height * 0.5)));
  const int max_v = std::min(
      static_cast<int>(height) - 1,
      static_cast<int>(std::floor(bbox.center_v + roi_height * 0.5)));
  if (min_u > max_u || min_v > max_v) {
    result.status = ProjectionStatus::kInvalidBoundingBox;
    return result;
  }

  std::vector<float> valid_depths;
  valid_depths.reserve(
      static_cast<std::size_t>(max_u - min_u + 1) *
      static_cast<std::size_t>(max_v - min_v + 1));
  for (int v = min_v; v <= max_v; ++v) {
    const std::size_t row_offset = static_cast<std::size_t>(v) * depth_image.step;
    for (int u = min_u; u <= max_u; ++u) {
      const std::size_t offset = row_offset + static_cast<std::size_t>(u) * bytes_per_pixel;
      const float depth = ReadFloat32(&depth_image.data[offset], depth_image.is_bigendian != 0U);
      if (std::isfinite(depth) && depth >= config_.min_depth && depth <= config_.max_depth) {
        valid_depths.push_back(depth);
      }
    }
  }

  result.valid_sample_count = valid_depths.size();
  if (valid_depths.size() < config_.min_valid_samples) {
    result.status = ProjectionStatus::kInsufficientValidDepth;
    return result;
  }
  result.depth = Median(&valid_depths);
  result.status = ProjectionStatus::kValid;
  return result;
}

ProjectionResult DepthProjector::Project(
    const BoundingBox2D& bbox, const sensor_msgs::msg::Image& depth_image,
    const sensor_msgs::msg::CameraInfo& camera_info) const {
  ProjectionResult result = SampleDepth(bbox, depth_image);
  if (!result.valid()) {
    return result;
  }
  const auto intrinsics = ParseIntrinsics(camera_info);
  if (!intrinsics || intrinsics->image_width != depth_image.width ||
      intrinsics->image_height != depth_image.height) {
    result.status = ProjectionStatus::kInvalidIntrinsics;
    result.point = geometry_msgs::msg::Point();
    result.depth = 0.0;
    return result;
  }

  result.point.x = (bbox.center_u - intrinsics->cx) * result.depth / intrinsics->fx;
  result.point.y = (bbox.center_v - intrinsics->cy) * result.depth / intrinsics->fy;
  result.point.z = result.depth;
  if (!IsFinite(result.point.x) || !IsFinite(result.point.y) || !IsFinite(result.point.z)) {
    result.status = ProjectionStatus::kInvalidIntrinsics;
    result.point = geometry_msgs::msg::Point();
    result.depth = 0.0;
  }
  return result;
}

}  // namespace robot_perception
