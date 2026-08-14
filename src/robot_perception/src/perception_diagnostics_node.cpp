#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces_perception/msg/detected_object3_d.hpp"
#include "robot_interfaces_perception/msg/perception_safety_event.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "vision_msgs/msg/detection2_d_array.hpp"

#include "robot_perception/perception_health.hpp"

namespace {

using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
using KeyValue = diagnostic_msgs::msg::KeyValue;

KeyValue Value(const std::string & key, const std::string & value)
{
  KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

std::string Number(const double value)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(4) << value;
  return stream.str();
}

double AgeSec(const rclcpp::Time & now, const rclcpp::Time & stamp)
{
  if (stamp.nanoseconds() == 0) {
    return std::numeric_limits<double>::infinity();
  }
  return std::max(0.0, (now - stamp).seconds());
}

}  // namespace

class PerceptionDiagnosticsNode final : public rclcpp::Node {
public:
  PerceptionDiagnosticsNode()
  : Node("perception_diagnostics_node"), tf_buffer_(get_clock())
  {
    update_rate_hz_ = declare_parameter<double>("update_rate_hz", 2.0);
    startup_grace_sec_ = declare_parameter<double>("startup_grace_sec", 5.0);
    camera_stale_warn_sec_ = declare_parameter<double>("camera_stale_warn_sec", 0.5);
    camera_stale_error_sec_ = declare_parameter<double>("camera_stale_error_sec", 1.5);
    detector_required_ = declare_parameter<bool>("detector_required", false);
    detector_stale_error_sec_ = declare_parameter<double>("detector_stale_error_sec", 3.0);
    tf_failure_warn_count_ = declare_parameter<int>("tf_failure_warn_count", 2);
    tf_failure_error_count_ = declare_parameter<int>("tf_failure_error_count", 5);
    depth_invalid_ratio_warn_ = declare_parameter<double>("depth_invalid_ratio_warn", 0.25);
    depth_invalid_ratio_error_ = declare_parameter<double>("depth_invalid_ratio_error", 0.60);
    depth_min_m_ = declare_parameter<double>("depth_min_m", 0.2);
    depth_max_m_ = declare_parameter<double>("depth_max_m", 8.0);
    rgb_topic_ = declare_parameter<std::string>("rgb_topic", "/camera/color/image_raw");
    depth_topic_ = declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw");
    camera_info_topic_ = declare_parameter<std::string>(
      "camera_info_topic", "/camera/color/camera_info");
    detections_topic_ = declare_parameter<std::string>(
      "detections_topic", "/perception/detections_2d");
    camera_point_topic_ = declare_parameter<std::string>(
      "camera_point_topic", "/perception/geometry/camera_point");
    objects_topic_ = declare_parameter<std::string>(
      "objects_topic", "/perception/objects_3d");
    safety_event_topic_ = declare_parameter<std::string>(
      "safety_event_topic", "/perception/safety_event");
    camera_frame_ = declare_parameter<std::string>(
      "camera_frame", "camera_color_optical_frame");
    target_frame_ = declare_parameter<std::string>("target_frame", "map");

    if (!std::isfinite(update_rate_hz_) || update_rate_hz_ <= 0.0) {
      throw std::invalid_argument("update_rate_hz must be positive");
    }
    if (camera_stale_error_sec_ <= camera_stale_warn_sec_ || depth_max_m_ <= depth_min_m_) {
      throw std::invalid_argument("diagnostic thresholds are inconsistent");
    }
    if (startup_grace_sec_ < 0.0 || tf_failure_warn_count_ < 1 ||
      tf_failure_error_count_ <= tf_failure_warn_count_)
    {
      throw std::invalid_argument("startup/TF thresholds are inconsistent");
    }
    if (depth_invalid_ratio_warn_ < 0.0 || depth_invalid_ratio_error_ > 1.0 ||
      depth_invalid_ratio_error_ <= depth_invalid_ratio_warn_)
    {
      throw std::invalid_argument("depth invalid ratio thresholds are inconsistent");
    }

    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/perception/diagnostics", rclcpp::QoS(10));
    auto sensor_qos = rclcpp::SensorDataQoS();
    rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
      rgb_topic_, sensor_qos, [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
        rgb_seen_ = true;
        rgb_receipt_ = now();
        rgb_stamp_ = rclcpp::Time(message->header.stamp);
        rgb_frame_ = message->header.frame_id;
      });
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, sensor_qos, [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
        depth_seen_ = true;
        depth_receipt_ = now();
        depth_stamp_ = rclcpp::Time(message->header.stamp);
        depth_frame_ = message->header.frame_id;
        EvaluateDepth(*message);
      });
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, sensor_qos, [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
        camera_info_seen_ = true;
        camera_info_receipt_ = now();
        camera_info_stamp_ = rclcpp::Time(message->header.stamp);
        camera_info_frame_ = message->header.frame_id;
        camera_info_valid_ = message->width > 0 && message->height > 0 &&
          std::isfinite(message->k[0]) && std::isfinite(message->k[4]) &&
          std::isfinite(message->k[2]) && std::isfinite(message->k[5]) &&
          message->k[0] > 0.0 && message->k[4] > 0.0 && message->k[8] == 1.0;
      });
    detections_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
      detections_topic_, 10, [this](vision_msgs::msg::Detection2DArray::ConstSharedPtr message) {
        detector_output_seen_ = true;
        detector_output_receipt_ = now();
        detector_output_count_ = message->detections.size();
      });
    camera_point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      camera_point_topic_, 10, [this](geometry_msgs::msg::PointStamped::ConstSharedPtr message) {
        camera_point_seen_ = true;
        camera_point_receipt_ = now();
        EvaluateObservationTf(rclcpp::Time(message->header.stamp));
      });
    objects_sub_ = create_subscription<robot_interfaces_perception::msg::DetectedObject3D>(
      objects_topic_, 10, [this](robot_interfaces_perception::msg::DetectedObject3D::ConstSharedPtr) {
        object_seen_ = true;
        object_receipt_ = now();
      });
    safety_sub_ =
      create_subscription<robot_interfaces_perception::msg::PerceptionSafetyEvent>(
      safety_event_topic_, 10,
      [this](robot_interfaces_perception::msg::PerceptionSafetyEvent::ConstSharedPtr) {
        safety_seen_ = true;
        safety_receipt_ = now();
      });
    diagnostics_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/perception/diagnostics", 10,
      [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message) {
        for (const auto & status : message->status) {
          if (status.name == "perception/detector") {
            detector_status_ = status;
            detector_diagnostic_seen_ = true;
            detector_diagnostic_receipt_ = now();
          }
        }
      });

    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / update_rate_hz_)),
      [this]() { PublishDiagnostics(); });
    start_time_ = now();
  }

private:
  diagnostic_msgs::msg::DiagnosticStatus MakeStatus(
    const std::string & name, const std::uint8_t level, const std::string & message,
    std::vector<KeyValue> values = {}) const
  {
    DiagnosticStatus status;
    status.name = name;
    status.hardware_id = "factory_patrol_perception";
    status.level = level;
    status.message = message;
    status.values = std::move(values);
    return status;
  }

  std::uint8_t MissingLevel() const
  {
    return (now() - start_time_).seconds() < startup_grace_sec_
      ? DiagnosticStatus::OK : DiagnosticStatus::ERROR;
  }

  robot_perception::HealthStatus StreamStatus(const bool seen, const rclcpp::Time & receipt) const
  {
    if (!seen) {
      return {MissingLevel(), seen ? "fresh" : "waiting for stream"};
    }
    robot_perception::HealthThresholds thresholds;
    thresholds.stale_warn_sec = camera_stale_warn_sec_;
    thresholds.stale_error_sec = camera_stale_error_sec_;
    return robot_perception::EvaluateStreamHealth(true, AgeSec(now(), receipt), thresholds);
  }

  DiagnosticStatus CameraStatus() const
  {
    auto health = StreamStatus(rgb_seen_, rgb_receipt_);
    if (rgb_seen_ && (rgb_frame_ != camera_frame_ || rgb_stamp_.nanoseconds() == 0)) {
      health.level = DiagnosticStatus::ERROR;
      health.message = "RGB frame/timestamp invalid";
    }
    return MakeStatus(
      "perception/camera_rgb", health.level, health.message,
      {Value("age_sec", Number(AgeSec(now(), rgb_receipt_))),
        Value("frame_id", rgb_frame_), Value("topic", rgb_topic_)});
  }

  DiagnosticStatus DepthStatus() const
  {
    auto health = StreamStatus(depth_seen_, depth_receipt_);
    if (depth_seen_ && (depth_frame_ != camera_frame_ || depth_stamp_.nanoseconds() == 0)) {
      health.level = DiagnosticStatus::ERROR;
      health.message = "depth frame/timestamp invalid";
    }
    return MakeStatus(
      "perception/camera_depth", health.level, health.message,
      {Value("age_sec", Number(AgeSec(now(), depth_receipt_))),
        Value("frame_id", depth_frame_), Value("topic", depth_topic_)});
  }

  DiagnosticStatus CameraInfoStatus() const
  {
    auto health = StreamStatus(camera_info_seen_, camera_info_receipt_);
    if (health.level < DiagnosticStatus::ERROR && camera_info_seen_ && !camera_info_valid_) {
      health.level = DiagnosticStatus::ERROR;
      health.message = "CameraInfo intrinsics invalid";
    }
    if (health.level < DiagnosticStatus::ERROR && camera_info_seen_ &&
      camera_info_frame_ != camera_frame_)
    {
      health.level = DiagnosticStatus::ERROR;
      health.message = "CameraInfo frame mismatch";
    }
    if (health.level < DiagnosticStatus::ERROR && camera_info_seen_ &&
      camera_info_stamp_.nanoseconds() == 0)
    {
      health.level = DiagnosticStatus::ERROR;
      health.message = "CameraInfo timestamp invalid";
    }
    return MakeStatus(
      "perception/camera_info", health.level, health.message,
      {Value("age_sec", Number(AgeSec(now(), camera_info_receipt_))),
        Value("frame_id", camera_info_frame_), Value("intrinsics_valid", camera_info_valid_ ? "true" : "false"),
        Value("topic", camera_info_topic_)});
  }

  DiagnosticStatus DepthQualityStatus() const
  {
    const auto health = robot_perception::EvaluateInvalidRatio(
      depth_quality_seen_, depth_invalid_ratio_, depth_invalid_ratio_warn_,
      depth_invalid_ratio_error_);
    std::uint8_t level = health.level;
    std::string message = health.message;
    if (!depth_quality_seen_ && (now() - start_time_).seconds() < startup_grace_sec_) {
      level = DiagnosticStatus::OK;
      message = "startup grace";
    }
    return MakeStatus(
      "perception/depth_quality", level, message,
      {Value("invalid_ratio", Number(depth_invalid_ratio_)),
        Value("valid_samples", std::to_string(depth_valid_samples_)),
        Value("invalid_samples", std::to_string(depth_invalid_samples_)),
        Value("encoding", depth_encoding_), Value("min_depth_m", Number(depth_min_m_)),
        Value("max_depth_m", Number(depth_max_m_))});
  }

  DiagnosticStatus DetectorStatus() const
  {
    if (detector_diagnostic_seen_) {
      auto status = detector_status_;
      if (detector_required_ &&
        AgeSec(now(), detector_diagnostic_receipt_) > detector_stale_error_sec_)
      {
        status.level = DiagnosticStatus::ERROR;
        status.message = "detector diagnostics stale";
      }
      return status;
    }
    if (!detector_required_) {
      return MakeStatus("perception/detector", DiagnosticStatus::OK, "detector not required");
    }
    if ((now() - start_time_).seconds() < startup_grace_sec_) {
      return MakeStatus("perception/detector", DiagnosticStatus::OK, "startup grace");
    }
    return MakeStatus("perception/detector", DiagnosticStatus::ERROR, "detector diagnostics missing");
  }

  DiagnosticStatus TfStatus() const
  {
    std::uint8_t level = DiagnosticStatus::OK;
    std::string message = tf_seen_ ? "observation-time TF available" : "waiting for observations";
    if (tf_seen_ && tf_consecutive_failures_ >= tf_failure_error_count_) {
      level = DiagnosticStatus::ERROR;
      message = "observation-time TF failures";
    } else if (tf_seen_ && tf_consecutive_failures_ >= tf_failure_warn_count_) {
      level = DiagnosticStatus::WARN;
      message = "observation-time TF intermittently unavailable";
    } else if (!tf_seen_ && camera_point_seen_ &&
      (now() - start_time_).seconds() >= startup_grace_sec_)
    {
      level = DiagnosticStatus::WARN;
      message = "no observation-time TF result";
    }
    return MakeStatus(
      "perception/tf", level, message,
      {Value("target_frame", target_frame_), Value("camera_frame", camera_frame_),
        Value("consecutive_failures", std::to_string(tf_consecutive_failures_)),
        Value("lookup_count", std::to_string(tf_lookup_count_)),
        Value("observation_age_sec", Number(AgeSec(now(), camera_point_receipt_)))});
  }

  void PublishDiagnostics()
  {
    const auto camera = CameraStatus();
    const auto depth = DepthStatus();
    const auto info = CameraInfoStatus();
    const auto detector = DetectorStatus();
    const auto tf = TfStatus();
    const auto quality = DepthQualityStatus();
    const std::vector<DiagnosticStatus> statuses{camera, depth, info, detector, tf, quality};
    std::uint8_t worst = DiagnosticStatus::OK;
    std::string message = "pipeline nominal";
    for (const auto & status : statuses) {
      if (status.level > worst) {
        worst = status.level;
        message = status.name + ": " + status.message;
      }
    }
    auto pipeline = MakeStatus("perception/pipeline", worst, message,
      {Value("camera_rgb", std::to_string(camera.level)),
        Value("camera_depth", std::to_string(depth.level)),
        Value("camera_info", std::to_string(info.level)),
        Value("detector", std::to_string(detector.level)), Value("tf", std::to_string(tf.level)),
        Value("depth_quality", std::to_string(quality.level)),
        Value("detector_output_seen", detector_output_seen_ ? "true" : "false"),
        Value("detector_output_age_sec", Number(AgeSec(now(), detector_output_receipt_))),
        Value("last_detection_count", std::to_string(detector_output_count_)),
        Value("objects_seen", object_seen_ ? "true" : "false"),
        Value("object_age_sec", Number(AgeSec(now(), object_receipt_))),
        Value("safety_event_seen", safety_seen_ ? "true" : "false"),
        Value("safety_event_age_sec", Number(AgeSec(now(), safety_receipt_)))});

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    array.status = statuses;
    array.status.push_back(std::move(pipeline));
    diagnostics_pub_->publish(array);
  }

  void EvaluateObservationTf(const rclcpp::Time & stamp)
  {
    if (stamp.nanoseconds() == 0) {
      return;
    }
    ++tf_lookup_count_;
    try {
      (void)tf_buffer_.lookupTransform(
        target_frame_, camera_frame_, stamp, tf2::durationFromSec(0.20));
      tf_seen_ = true;
      tf_consecutive_failures_ = 0;
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 5000, "observation-time TF available %s <- %s",
        target_frame_.c_str(), camera_frame_.c_str());
    } catch (const tf2::TransformException & error) {
      tf_seen_ = true;
      ++tf_consecutive_failures_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "observation-time TF unavailable %s <- %s: %s",
        target_frame_.c_str(), camera_frame_.c_str(), error.what());
    }
  }

  void EvaluateDepth(const sensor_msgs::msg::Image & image)
  {
    depth_encoding_ = image.encoding;
    std::size_t total = 0;
    std::size_t invalid = 0;
    const std::size_t pixel_count = static_cast<std::size_t>(image.width) * image.height;
    const std::size_t stride = std::max<std::size_t>(1, pixel_count / 5000U);
    for (std::size_t index = 0; index < pixel_count; index += stride) {
      double value = std::numeric_limits<double>::quiet_NaN();
      const std::size_t row = index / image.width;
      const std::size_t column = index % image.width;
      if (image.encoding == sensor_msgs::image_encodings::TYPE_32FC1 &&
        row * image.step + (column + 1U) * sizeof(float) <= image.data.size())
      {
        float raw = 0.0F;
        std::memcpy(&raw, image.data.data() + row * image.step + column * sizeof(float), sizeof(raw));
        value = static_cast<double>(raw);
      } else if (image.encoding == sensor_msgs::image_encodings::TYPE_16UC1 &&
        row * image.step + (column + 1U) * sizeof(std::uint16_t) <= image.data.size())
      {
        std::uint16_t raw = 0U;
        std::memcpy(&raw, image.data.data() + row * image.step + column * sizeof(std::uint16_t), sizeof(raw));
        value = static_cast<double>(raw) / 1000.0;
      }
      ++total;
      if (!std::isfinite(value) || value <= 0.0 || value < depth_min_m_ || value > depth_max_m_) {
        ++invalid;
      }
    }
    depth_valid_samples_ = total - invalid;
    depth_invalid_samples_ = invalid;
    depth_invalid_ratio_ = total == 0U ? 1.0 : static_cast<double>(invalid) / total;
    depth_quality_seen_ = true;
  }

  double update_rate_hz_{2.0};
  double startup_grace_sec_{5.0};
  double camera_stale_warn_sec_{0.5};
  double camera_stale_error_sec_{1.5};
  bool detector_required_{false};
  double detector_stale_error_sec_{3.0};
  int tf_failure_warn_count_{2};
  int tf_failure_error_count_{5};
  double depth_invalid_ratio_warn_{0.25};
  double depth_invalid_ratio_error_{0.60};
  double depth_min_m_{0.2};
  double depth_max_m_{8.0};
  std::string rgb_topic_, depth_topic_, camera_info_topic_, detections_topic_;
  std::string camera_point_topic_, objects_topic_, safety_event_topic_;
  std::string camera_frame_, target_frame_;
  bool rgb_seen_{false}, depth_seen_{false}, camera_info_seen_{false};
  bool camera_info_valid_{false};
  bool detector_output_seen_{false}, detector_diagnostic_seen_{false};
  bool camera_point_seen_{false}, object_seen_{false}, safety_seen_{false};
  bool depth_quality_seen_{false}, tf_seen_{false};
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time rgb_receipt_{0, 0, RCL_ROS_TIME}, depth_receipt_{0, 0, RCL_ROS_TIME};
  rclcpp::Time camera_info_receipt_{0, 0, RCL_ROS_TIME};
  rclcpp::Time detector_diagnostic_receipt_{0, 0, RCL_ROS_TIME};
  rclcpp::Time detector_output_receipt_{0, 0, RCL_ROS_TIME};
  rclcpp::Time camera_point_receipt_{0, 0, RCL_ROS_TIME};
  rclcpp::Time object_receipt_{0, 0, RCL_ROS_TIME}, safety_receipt_{0, 0, RCL_ROS_TIME};
  rclcpp::Time rgb_stamp_{0, 0, RCL_ROS_TIME}, depth_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time camera_info_stamp_{0, 0, RCL_ROS_TIME};
  std::string rgb_frame_, depth_frame_, camera_info_frame_, depth_encoding_;
  double depth_invalid_ratio_{1.0};
  std::size_t depth_valid_samples_{0U}, depth_invalid_samples_{0U}, detector_output_count_{0U};
  int tf_consecutive_failures_{0}, tf_lookup_count_{0};
  DiagnosticStatus detector_status_;
  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_, depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr camera_point_sub_;
  rclcpp::Subscription<robot_interfaces_perception::msg::DetectedObject3D>::SharedPtr objects_sub_;
  rclcpp::Subscription<robot_interfaces_perception::msg::PerceptionSafetyEvent>::SharedPtr safety_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerceptionDiagnosticsNode>());
  rclcpp::shutdown();
  return 0;
}
