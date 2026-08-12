#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "rmw/qos_profiles.h"
#include "robot_perception/depth_projector.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

namespace robot_perception {

class GeometryValidationNode final : public rclcpp::Node {
 public:
  GeometryValidationNode() : Node("geometry_validation_node") {
    rgb_topic_ = declare_parameter<std::string>("rgb_topic", "/camera/color/image_raw");
    depth_topic_ = declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw");
    camera_info_topic_ =
        declare_parameter<std::string>("camera_info_topic", "/camera/color/camera_info");
    camera_frame_ =
        declare_parameter<std::string>("camera_frame", "camera_color_optical_frame");
    target_frame_ = declare_parameter<std::string>("target_frame", "map");
    camera_point_topic_ = declare_parameter<std::string>(
        "camera_point_topic", "/perception/geometry/camera_point");
    map_point_topic_ =
        declare_parameter<std::string>("map_point_topic", "/perception/geometry/map_point");
    marker_topic_ =
        declare_parameter<std::string>("marker_topic", "/perception/markers");
    detection_topic_ = declare_parameter<std::string>(
        "detection_topic", "/perception/detections_2d");
    geometry_input_mode_ =
        declare_parameter<std::string>("geometry_input_mode", "synthetic");

    DepthSamplingConfig depth_config;
    depth_config.min_depth = declare_parameter<double>("min_depth", 0.2);
    depth_config.max_depth = declare_parameter<double>("max_depth", 8.0);
    depth_config.roi_ratio = declare_parameter<double>("roi_ratio", 0.3);
    const auto min_valid_samples =
        declare_parameter<std::int64_t>("min_valid_samples", 5);
    if (min_valid_samples <= 0) {
      throw std::invalid_argument("min_valid_samples must be positive");
    }
    if (!std::isfinite(depth_config.min_depth) ||
        !std::isfinite(depth_config.max_depth) ||
        !std::isfinite(depth_config.roi_ratio) || depth_config.min_depth <= 0.0 ||
        depth_config.max_depth <= depth_config.min_depth ||
        depth_config.roi_ratio <= 0.0 || depth_config.roi_ratio > 1.0) {
      throw std::invalid_argument(
          "depth limits and roi_ratio must form a finite valid sampling policy");
    }
    depth_config.min_valid_samples = static_cast<std::size_t>(min_valid_samples);
    projector_ = std::make_unique<DepthProjector>(depth_config);

    bbox_.center_u = declare_parameter<double>("synthetic_bbox.center_u", 320.0);
    bbox_.center_v = declare_parameter<double>("synthetic_bbox.center_v", 240.0);
    bbox_.width = declare_parameter<double>("synthetic_bbox.width", 80.0);
    bbox_.height = declare_parameter<double>("synthetic_bbox.height", 80.0);

    const auto sync_queue_size =
        declare_parameter<std::int64_t>("sync_queue_size", 10);
    if (sync_queue_size < 2 ||
        sync_queue_size > static_cast<std::int64_t>(std::numeric_limits<int>::max())) {
      throw std::invalid_argument("sync_queue_size must be between 2 and INT_MAX");
    }
    sync_queue_size_ = static_cast<int>(sync_queue_size);
    sync_slop_sec_ = declare_parameter<double>("sync_slop_sec", 0.05);
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.1);
    marker_scale_ = declare_parameter<double>("marker_scale", 0.18);
    marker_namespace_ =
        declare_parameter<std::string>("marker_namespace", "geometry_validation");
    marker_lifetime_sec_ = declare_parameter<double>("marker_lifetime_sec", 0.75);
    ground_truth_enabled_ = declare_parameter<bool>("ground_truth.enabled", true);
    ground_truth_x_ = declare_parameter<double>("ground_truth.x", 2.70);
    ground_truth_y_ = declare_parameter<double>("ground_truth.y", 0.0);
    ground_truth_z_ = declare_parameter<double>("ground_truth.z", 0.495);

    if (camera_frame_.empty() || target_frame_.empty()) {
      throw std::invalid_argument("camera_frame and target_frame must not be empty");
    }
    if (geometry_input_mode_ != "synthetic" && geometry_input_mode_ != "detector") {
      throw std::invalid_argument(
          "geometry_input_mode must be either synthetic or detector");
    }
    if (!std::isfinite(sync_slop_sec_) || sync_slop_sec_ <= 0.0 ||
        !std::isfinite(tf_timeout_sec_) || tf_timeout_sec_ < 0.0) {
      throw std::invalid_argument(
          "sync_slop_sec must be positive and tf_timeout_sec must be nonnegative");
    }
    if (!std::isfinite(marker_scale_) || marker_scale_ <= 0.0 ||
        !std::isfinite(marker_lifetime_sec_) || marker_lifetime_sec_ <= 0.0) {
      throw std::invalid_argument(
          "marker_scale and marker_lifetime_sec must be positive and finite");
    }

    camera_point_pub_ =
        create_publisher<geometry_msgs::msg::PointStamped>(camera_point_topic_, 10);
    map_point_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(map_point_topic_, 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(marker_topic_, 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    depth_sub_.subscribe(this, depth_topic_, rmw_qos_profile_sensor_data);
    camera_info_sub_.subscribe(this, camera_info_topic_, rmw_qos_profile_sensor_data);
    if (geometry_input_mode_ == "synthetic") {
      rgb_sub_.subscribe(this, rgb_topic_, rmw_qos_profile_sensor_data);
      SyntheticSyncPolicy policy(static_cast<std::uint32_t>(sync_queue_size_));
      policy.setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_slop_sec_));
      synthetic_synchronizer_ = std::make_unique<SyntheticSynchronizer>(policy);
      synthetic_synchronizer_->connectInput(rgb_sub_, depth_sub_, camera_info_sub_);
      synthetic_synchronizer_->registerCallback(std::bind(
          &GeometryValidationNode::HandleSyntheticObservation, this,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    } else {
      detection_sub_.subscribe(this, detection_topic_, rmw_qos_profile_sensor_data);
      DetectorSyncPolicy policy(static_cast<std::uint32_t>(sync_queue_size_));
      policy.setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_slop_sec_));
      detector_synchronizer_ = std::make_unique<DetectorSynchronizer>(policy);
      detector_synchronizer_->connectInput(
          detection_sub_, depth_sub_, camera_info_sub_);
      detector_synchronizer_->registerCallback(std::bind(
          &GeometryValidationNode::HandleDetectorObservation, this,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    }

    RCLCPP_INFO(
        get_logger(),
        "geometry validation ready: mode=%s, target_frame=%s, observation-time TF only",
        geometry_input_mode_.c_str(), target_frame_.c_str());
  }

 private:
  using SyntheticSyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;
  using SyntheticSynchronizer = message_filters::Synchronizer<SyntheticSyncPolicy>;
  using DetectorSyncPolicy = message_filters::sync_policies::ApproximateTime<
      vision_msgs::msg::Detection2DArray, sensor_msgs::msg::Image,
      sensor_msgs::msg::CameraInfo>;
  using DetectorSynchronizer = message_filters::Synchronizer<DetectorSyncPolicy>;

  void HandleSyntheticObservation(
      const sensor_msgs::msg::Image::ConstSharedPtr& rgb,
      const sensor_msgs::msg::Image::ConstSharedPtr& depth,
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info) {
    if (rgb->header.frame_id != camera_frame_ ||
        depth->header.frame_id != camera_frame_ ||
        camera_info->header.frame_id != camera_frame_) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "camera stream frame mismatch; expected %s and suppressing geometry output",
          camera_frame_.c_str());
      return;
    }
    if (depth->header.stamp.sec == 0 && depth->header.stamp.nanosec == 0U) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "depth observation has a zero timestamp; suppressing geometry output");
      return;
    }
    if (rgb->width != depth->width || rgb->height != depth->height) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "RGB and depth dimensions differ; suppressing geometry output");
      return;
    }

    ClearMarkers(depth->header.stamp);
    ProjectAndPublish(
        bbox_, "synthetic", 1.0, depth->header.stamp, *depth, *camera_info, 0U,
        ground_truth_enabled_);
  }

  void HandleDetectorObservation(
      const vision_msgs::msg::Detection2DArray::ConstSharedPtr& detections,
      const sensor_msgs::msg::Image::ConstSharedPtr& depth,
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info) {
    if (detections->header.frame_id != camera_frame_ ||
        depth->header.frame_id != camera_frame_ ||
        camera_info->header.frame_id != camera_frame_) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "detector/depth frame mismatch; expected %s and suppressing geometry output",
          camera_frame_.c_str());
      return;
    }
    if (detections->header.stamp.sec == 0 &&
        detections->header.stamp.nanosec == 0U) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "detection observation has a zero timestamp; suppressing geometry output");
      return;
    }

    ClearMarkers(detections->header.stamp);
    std::size_t marker_index = 0U;
    for (const auto& detection : detections->detections) {
      if (detection.results.empty()) {
        continue;
      }
      const auto best = std::max_element(
          detection.results.begin(), detection.results.end(),
          [](const auto& left, const auto& right) {
            return left.hypothesis.score < right.hypothesis.score;
          });
      BoundingBox2D bbox;
      bbox.center_u = detection.bbox.center.position.x;
      bbox.center_v = detection.bbox.center.position.y;
      bbox.width = detection.bbox.size_x;
      bbox.height = detection.bbox.size_y;
      ProjectAndPublish(
          bbox, best->hypothesis.class_id, best->hypothesis.score,
          detections->header.stamp, *depth, *camera_info, marker_index, false);
      ++marker_index;
    }
  }

  void ProjectAndPublish(
      const BoundingBox2D& bbox, const std::string& class_name,
      const double confidence, const builtin_interfaces::msg::Time& observation_stamp,
      const sensor_msgs::msg::Image& depth,
      const sensor_msgs::msg::CameraInfo& camera_info,
      const std::size_t marker_index, const bool report_ground_truth) {
    const auto result = projector_->Project(bbox, depth, camera_info);
    if (!result.valid()) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "depth projection rejected observation: %s (valid samples=%zu)",
          ProjectionStatusName(result.status), result.valid_sample_count);
      return;
    }

    geometry_msgs::msg::PointStamped camera_point;
    camera_point.header.stamp = observation_stamp;
    camera_point.header.frame_id = camera_frame_;
    camera_point.point = result.point;
    last_sample_count_ = result.valid_sample_count;
    camera_point_pub_->publish(camera_point);

    try {
      const auto transform = tf_buffer_->lookupTransform(
          target_frame_, camera_frame_, rclcpp::Time(observation_stamp),
          tf2::durationFromSec(tf_timeout_sec_));
      geometry_msgs::msg::PointStamped map_point;
      tf2::doTransform(camera_point, map_point, transform);
      map_point.header.stamp = observation_stamp;
      map_point.header.frame_id = target_frame_;
      if (!PointIsFinite(map_point)) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "TF produced a non-finite map point; suppressing map output");
        return;
      }
      map_point_pub_->publish(map_point);
      marker_pub_->publish(BuildPointMarker(map_point, marker_index));
      marker_pub_->publish(BuildTextMarker(
          map_point, marker_index, class_name, confidence, result.depth));
      if (report_ground_truth) {
        ReportGroundTruthError(map_point);
      }
    } catch (const tf2::TransformException& error) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "observation-time TF %s <- %s failed: %s; map output suppressed",
          target_frame_.c_str(), camera_frame_.c_str(), error.what());
    }
  }

  static bool PointIsFinite(const geometry_msgs::msg::PointStamped& point) {
    return std::isfinite(point.point.x) && std::isfinite(point.point.y) &&
           std::isfinite(point.point.z);
  }

  void ClearMarkers(const builtin_interfaces::msg::Time& stamp) {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = target_frame_;
    marker.ns = marker_namespace_;
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_pub_->publish(marker);
  }

  visualization_msgs::msg::Marker BuildPointMarker(
      const geometry_msgs::msg::PointStamped& map_point,
      const std::size_t marker_index) const {
    visualization_msgs::msg::Marker marker;
    marker.header = map_point.header;
    marker.ns = marker_namespace_;
    marker.id = static_cast<std::int32_t>(marker_index * 2U);
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = map_point.point;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker_scale_;
    marker.scale.y = marker_scale_;
    marker.scale.z = marker_scale_;
    marker.color.r = 0.10F;
    marker.color.g = 0.95F;
    marker.color.b = 0.35F;
    marker.color.a = 1.0F;
    marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_sec_);
    return marker;
  }

  visualization_msgs::msg::Marker BuildTextMarker(
      const geometry_msgs::msg::PointStamped& map_point,
      const std::size_t marker_index, const std::string& class_name,
      const double confidence, const double depth) const {
    auto marker = BuildPointMarker(map_point, marker_index);
    marker.id = static_cast<std::int32_t>(marker_index * 2U + 1U);
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.pose.position.z += marker_scale_ * 1.5;
    marker.scale.x = 0.0;
    marker.scale.y = 0.0;
    marker.scale.z = marker_scale_;
    marker.color.r = 1.0F;
    marker.color.g = 1.0F;
    marker.color.b = 1.0F;
    char text[192];
    std::snprintf(
        text, sizeof(text), "%s %.2f depth=%.2fm map=(%.2f, %.2f, %.2f)",
        class_name.c_str(), confidence, depth, map_point.point.x, map_point.point.y,
        map_point.point.z);
    marker.text = text;
    return marker;
  }

  void ReportGroundTruthError(const geometry_msgs::msg::PointStamped& map_point) {
    if (!ground_truth_enabled_) {
      return;
    }
    const double dx = map_point.point.x - ground_truth_x_;
    const double dy = map_point.point.y - ground_truth_y_;
    const double dz = map_point.point.z - ground_truth_z_;
    const double error = std::sqrt(dx * dx + dy * dy + dz * dz);
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "geometry validation: P_est=[%.3f, %.3f, %.3f], P_gt=[%.3f, %.3f, %.3f], "
        "error=%.4f m, valid_depth_samples=%zu",
        map_point.point.x, map_point.point.y, map_point.point.z, ground_truth_x_,
        ground_truth_y_, ground_truth_z_, error, last_sample_count_);
  }

  std::string rgb_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string camera_frame_;
  std::string target_frame_;
  std::string camera_point_topic_;
  std::string map_point_topic_;
  std::string marker_topic_;
  std::string detection_topic_;
  std::string geometry_input_mode_;
  std::string marker_namespace_;
  BoundingBox2D bbox_;
  int sync_queue_size_ = 10;
  double sync_slop_sec_ = 0.05;
  double tf_timeout_sec_ = 0.1;
  double marker_scale_ = 0.18;
  double marker_lifetime_sec_ = 0.75;
  bool ground_truth_enabled_ = false;
  double ground_truth_x_ = 0.0;
  double ground_truth_y_ = 0.0;
  double ground_truth_z_ = 0.0;
  std::size_t last_sample_count_ = 0U;

  std::unique_ptr<DepthProjector> projector_;
  message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
  message_filters::Subscriber<vision_msgs::msg::Detection2DArray> detection_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_sub_;
  std::unique_ptr<SyntheticSynchronizer> synthetic_synchronizer_;
  std::unique_ptr<DetectorSynchronizer> detector_synchronizer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr camera_point_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr map_point_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

}  // namespace robot_perception

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_perception::GeometryValidationNode>());
  rclcpp::shutdown();
  return 0;
}
