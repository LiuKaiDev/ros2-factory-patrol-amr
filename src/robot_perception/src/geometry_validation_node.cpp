#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
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
#include "robot_interfaces_perception/msg/detected_object3_d.hpp"
#include "robot_interfaces_perception/msg/perception_event.hpp"
#include "robot_interfaces_perception/msg/perception_safety_event.hpp"
#include "robot_perception/depth_projector.hpp"
#include "robot_perception/inspection_event_policy.hpp"
#include "robot_perception/perception_safety_policy.hpp"
#include "robot_perception/perception_health.hpp"
#include "robot_perception/target_manager.hpp"
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
    objects_3d_topic_ = declare_parameter<std::string>(
        "objects_3d_topic", "/perception/objects_3d");
    event_topic_ =
        declare_parameter<std::string>("inspection.event_topic", "/perception/events");
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

    TargetManagerConfig tracking_config;
    const auto confirm_frames =
        declare_parameter<std::int64_t>("tracking.confirm_frames", 3);
    const auto lost_frames =
        declare_parameter<std::int64_t>("tracking.lost_frames", 5);
    const auto lost_retirement_frames =
        declare_parameter<std::int64_t>("tracking.lost_retirement_frames", 0);
    if (confirm_frames <= 0 || lost_frames <= 0 || lost_retirement_frames < 0) {
      throw std::invalid_argument(
          "tracking frame counts must be positive, with retirement zero or positive");
    }
    tracking_config.confirm_frames = static_cast<std::size_t>(confirm_frames);
    tracking_config.lost_frames = static_cast<std::size_t>(lost_frames);
    tracking_config.lost_retirement_frames =
        static_cast<std::size_t>(lost_retirement_frames);
    tracking_config.max_match_distance =
        declare_parameter<double>("tracking.max_match_distance", 0.5);
    tracking_config.ema_alpha =
        declare_parameter<double>("tracking.ema_alpha", 0.4);
    tracking_config.processed_cooldown_sec =
        declare_parameter<double>("tracking.processed_cooldown_sec", 10.0);
    target_manager_ = std::make_unique<TargetManager>(tracking_config);

    InspectionEventPolicyConfig inspection_config;
    inspection_config.enabled =
        declare_parameter<bool>("inspection.enabled", true);
    inspection_config.allowed_classes =
        declare_parameter<std::vector<std::string>>(
            "inspection.allowed_classes", {"chair"});
    inspection_config.min_confidence =
        declare_parameter<double>("inspection.min_confidence", 0.5);
    inspection_event_policy_ =
        std::make_unique<InspectionEventPolicy>(inspection_config);

    PerceptionSafetyConfig safety_config;
    safety_config.enabled = declare_parameter<bool>("safety.enabled", true);
    safety_config.person_class =
        declare_parameter<std::string>("safety.person_class", "person");
    safety_config.person_slow_distance =
        declare_parameter<double>("safety.person_slow_distance", 3.0);
    safety_config.person_stop_distance =
        declare_parameter<double>("safety.person_stop_distance", 1.5);
    const auto clear_observations =
        declare_parameter<std::int64_t>("safety.clear_observations", 3);
    if (clear_observations <= 0) {
      throw std::invalid_argument("safety.clear_observations must be positive");
    }
    safety_config.clear_observations =
        static_cast<std::size_t>(clear_observations);
    safety_config.stop_hysteresis =
        declare_parameter<double>("safety.stop_hysteresis", 0.2);
    safety_config.slow_hysteresis =
        declare_parameter<double>("safety.slow_hysteresis", 0.2);
    safety_config.max_target_age_sec =
        declare_parameter<double>("safety.max_target_age_sec", 2.5);
    safety_event_topic_ = declare_parameter<std::string>(
        "safety.event_topic", "/perception/safety_event");
    safety_robot_frame_ =
        declare_parameter<std::string>("safety.robot_frame", "base_link");
    safety_map_name_ =
        declare_parameter<std::string>("safety.map_name", "factory_patrol");
    safety_danger_zone_enabled_ =
        declare_parameter<bool>("safety.danger_zone_enabled", false);
    safety_zones_file_ =
        declare_parameter<std::string>("safety.zones_file", "");
    safety_policy_ = std::make_unique<PerceptionSafetyPolicy>(safety_config);
    if (safety_danger_zone_enabled_) {
      if (safety_zones_file_.empty()) {
        throw std::invalid_argument(
            "safety.zones_file is required when danger zones are enabled");
      }
      robot_navigation::ZoneCatalog zone_catalog(safety_zones_file_);
      for (const auto& zone : zone_catalog.ForMap(safety_map_name_, false)) {
        if (zone.type == "danger_zone") {
          if (zone.frame_id != target_frame_) {
            throw std::invalid_argument(
                "perception danger zones must use the geometry target frame");
          }
          danger_zones_.push_back(zone);
        }
      }
    }

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

    if (camera_frame_.empty() || target_frame_.empty() ||
        safety_robot_frame_.empty() || safety_event_topic_.empty()) {
      throw std::invalid_argument(
          "camera, target, safety robot frames and safety event topic must not be empty");
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
    objects_3d_pub_ =
        create_publisher<robot_interfaces_perception::msg::DetectedObject3D>(
            objects_3d_topic_, 10);
    const auto event_qos = rclcpp::QoS(10).reliable().transient_local();
    event_pub_ = create_publisher<
        robot_interfaces_perception::msg::PerceptionEvent>(event_topic_, event_qos);
    safety_event_pub_ = create_publisher<
        robot_interfaces_perception::msg::PerceptionSafetyEvent>(
            safety_event_topic_, rclcpp::QoS(10).reliable());
    event_sub_ = create_subscription<
        robot_interfaces_perception::msg::PerceptionEvent>(
        event_topic_, event_qos,
        [this](const robot_interfaces_perception::msg::PerceptionEvent::SharedPtr event) {
          HandlePerceptionEvent(*event);
        });

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
        "geometry validation ready: mode=%s, target_frame=%s, observation-time TF only, "
        "managed_targets=%s, perception_safety=%s",
        geometry_input_mode_.c_str(), target_frame_.c_str(), objects_3d_topic_.c_str(),
        safety_event_topic_.c_str());
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
        ground_truth_enabled_, true);
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

    std::vector<TargetObservation> observations;
    observations.reserve(detections->detections.size());
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
      const auto observation = ProjectAndPublish(
          bbox, best->hypothesis.class_id, best->hypothesis.score,
          detections->header.stamp, *depth, *camera_info, 0U, false, false);
      if (observation) {
        observations.push_back(*observation);
      }
    }

    const auto update_start = std::chrono::steady_clock::now();
    const auto& targets = target_manager_->Update(
        observations, rclcpp::Time(detections->header.stamp).nanoseconds());
    const auto update_end = std::chrono::steady_clock::now();
    target_manager_latency_total_us_ +=
        std::chrono::duration<double, std::micro>(update_end - update_start).count();
    ++target_manager_update_count_;
    if (target_manager_update_count_ == 1U ||
        target_manager_update_count_ % 30U == 0U) {
      RCLCPP_INFO(
          get_logger(),
          "TargetManager latency: current=%.3f us, average=%.3f us, "
          "observations=%zu, retained_targets=%zu",
          std::chrono::duration<double, std::micro>(update_end - update_start).count(),
          target_manager_latency_total_us_ /
              static_cast<double>(target_manager_update_count_),
          observations.size(), targets.size());
    }
    PublishManagedTargets(targets, detections->header.stamp);
    if (HasValidSafetyObservation(detections->detections.size(), observations.size())) {
      EvaluateAndPublishSafety(targets, detections->header.stamp);
    } else {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "detector input has no valid projected observations; no fresh perception safety event published");
    }
  }

  std::optional<TargetObservation> ProjectAndPublish(
      const BoundingBox2D& bbox, const std::string& class_name,
      const double confidence, const builtin_interfaces::msg::Time& observation_stamp,
      const sensor_msgs::msg::Image& depth,
      const sensor_msgs::msg::CameraInfo& camera_info,
      const std::size_t marker_index, const bool report_ground_truth,
      const bool publish_raw_marker) {
    const auto result = projector_->Project(bbox, depth, camera_info);
    if (!result.valid()) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "depth projection rejected observation: %s (valid samples=%zu)",
          ProjectionStatusName(result.status), result.valid_sample_count);
      return std::nullopt;
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
        return std::nullopt;
      }
      map_point_pub_->publish(map_point);
      if (publish_raw_marker) {
        marker_pub_->publish(BuildRawPointMarker(map_point, marker_index));
        marker_pub_->publish(BuildRawTextMarker(
            map_point, marker_index, class_name, confidence, result.depth));
      }
      if (report_ground_truth) {
        ReportGroundTruthError(map_point);
      }
      TargetObservation observation;
      observation.class_name = class_name;
      observation.confidence = confidence;
      observation.position = {
          map_point.point.x, map_point.point.y, map_point.point.z};
      observation.timestamp_ns = rclcpp::Time(observation_stamp).nanoseconds();
      observation.depth_valid = true;
      return observation;
    } catch (const tf2::TransformException& error) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "observation-time TF %s <- %s failed: %s; map output suppressed",
          target_frame_.c_str(), camera_frame_.c_str(), error.what());
      return std::nullopt;
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

  visualization_msgs::msg::Marker BuildRawPointMarker(
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

  visualization_msgs::msg::Marker BuildRawTextMarker(
      const geometry_msgs::msg::PointStamped& map_point,
      const std::size_t marker_index, const std::string& class_name,
      const double confidence, const double depth) const {
    auto marker = BuildRawPointMarker(map_point, marker_index);
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

  void PublishManagedTargets(
      const std::vector<ManagedTarget>& targets,
      const builtin_interfaces::msg::Time& update_stamp) {
    ClearMarkers(update_stamp);
    for (const auto& target : targets) {
      robot_interfaces_perception::msg::DetectedObject3D message;
      message.header.stamp = rclcpp::Time(target.last_observation_ns);
      message.header.frame_id = target_frame_;
      message.target_id = target.target_id;
      message.class_name = target.class_name;
      message.confidence = static_cast<float>(target.confidence);
      message.position.x = target.filtered_position.x;
      message.position.y = target.filtered_position.y;
      message.position.z = target.filtered_position.z;
      message.depth_valid = target.depth_valid;
      message.tracking_state = static_cast<std::uint8_t>(target.state);
      objects_3d_pub_->publish(message);
      marker_pub_->publish(BuildManagedPointMarker(message));
      marker_pub_->publish(BuildManagedTextMarker(message));
      if (inspection_event_policy_->ShouldEmit(target)) {
        PublishInspectionRequired(target, update_stamp);
      }
    }
  }

  void EvaluateAndPublishSafety(
      const std::vector<ManagedTarget>& targets,
      const builtin_interfaces::msg::Time& update_stamp) {
    Position3D robot_position;
    try {
      const auto transform = tf_buffer_->lookupTransform(
          target_frame_, safety_robot_frame_, rclcpp::Time(update_stamp),
          tf2::durationFromSec(tf_timeout_sec_));
      robot_position = {
          transform.transform.translation.x,
          transform.transform.translation.y,
          transform.transform.translation.z};
    } catch (const tf2::TransformException& error) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "perception safety TF %s <- %s failed: %s; no safety event published",
          target_frame_.c_str(), safety_robot_frame_.c_str(), error.what());
      return;
    }
    if (!std::isfinite(robot_position.x) || !std::isfinite(robot_position.y) ||
        !std::isfinite(robot_position.z)) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "perception safety robot pose is non-finite; no safety event published");
      return;
    }

    std::vector<SafetyTarget> safety_targets;
    safety_targets.reserve(targets.size());
    for (const auto& target : targets) {
      SafetyTarget safety_target;
      safety_target.target_id = target.target_id;
      safety_target.class_name = target.class_name;
      safety_target.position = target.filtered_position;
      safety_target.state = target.state;
      safety_target.last_observation_ns = target.last_observation_ns;
      safety_target.missed_frames = target.missed_frames;
      safety_target.depth_valid = target.depth_valid;
      safety_targets.push_back(std::move(safety_target));
    }

    const auto& decision = safety_policy_->Evaluate(
        safety_targets, robot_position, danger_zones_,
        rclcpp::Time(update_stamp).nanoseconds());
    if (!decision.input_valid) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "perception safety input invalid; retaining policy state without publishing");
      return;
    }
    PublishSafetyEvent(decision, update_stamp);
    PublishSafetyMarkers(decision, robot_position, update_stamp);
  }

  void PublishSafetyEvent(
      const PerceptionSafetyDecision& decision,
      const builtin_interfaces::msg::Time& update_stamp) {
    using SafetyEvent = robot_interfaces_perception::msg::PerceptionSafetyEvent;
    SafetyEvent event;
    event.header.stamp = update_stamp;
    event.header.frame_id = target_frame_;
    event.target_id = decision.target_id;
    event.class_name = decision.class_name;
    event.event_type = PerceptionSafetyReasonName(decision.reason);
    event.safety_state = static_cast<std::uint8_t>(decision.level);
    event.severity = decision.level == PerceptionSafetyLevel::kStop
                         ? SafetyEvent::SEVERITY_CRITICAL
                         : decision.level == PerceptionSafetyLevel::kSpeedLimited
                               ? SafetyEvent::SEVERITY_WARNING
                               : SafetyEvent::SEVERITY_INFO;
    event.target_position.x = decision.target_position.x;
    event.target_position.y = decision.target_position.y;
    event.target_position.z = decision.target_position.z;
    event.distance_m = static_cast<float>(decision.distance_m);
    event.zone_id = decision.zone_id;
    event.source = "robot_perception";
    event.reason = "PERCEPTION_" + std::string(PerceptionSafetyReasonName(decision.reason));
    safety_event_pub_->publish(event);
  }

  void PublishSafetyMarkers(
      const PerceptionSafetyDecision& decision, const Position3D& robot_position,
      const builtin_interfaces::msg::Time& update_stamp) {
    for (std::size_t index = 0U; index < danger_zones_.size(); ++index) {
      const auto& zone = danger_zones_[index];
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = update_stamp;
      marker.header.frame_id = target_frame_;
      marker.ns = "perception_safety_zones";
      marker.id = static_cast<std::int32_t>(index);
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.05;
      marker.color.r = 0.95F;
      marker.color.g = 0.15F;
      marker.color.b = 0.10F;
      marker.color.a = 0.90F;
      for (std::size_t point = 0U; point < zone.polygon_x.size(); ++point) {
        geometry_msgs::msg::Point position;
        position.x = zone.polygon_x[point];
        position.y = zone.polygon_y[point];
        position.z = 0.08;
        marker.points.push_back(position);
      }
      if (!marker.points.empty()) {
        marker.points.push_back(marker.points.front());
      }
      marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_sec_);
      marker_pub_->publish(marker);
    }

    visualization_msgs::msg::Marker status;
    status.header.stamp = update_stamp;
    status.header.frame_id = target_frame_;
    status.ns = "perception_safety_status";
    status.id = 0;
    status.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    status.action = visualization_msgs::msg::Marker::ADD;
    status.pose.position.x = decision.target_id == 0U
                                 ? robot_position.x
                                 : decision.target_position.x;
    status.pose.position.y = decision.target_id == 0U
                                 ? robot_position.y
                                 : decision.target_position.y;
    status.pose.position.z = decision.target_id == 0U
                                 ? robot_position.z + 1.0
                                 : decision.target_position.z + 0.5;
    status.pose.orientation.w = 1.0;
    status.scale.z = marker_scale_;
    status.color.r = 1.0F;
    status.color.g = decision.level == PerceptionSafetyLevel::kStop ? 0.15F : 0.85F;
    status.color.b = decision.level == PerceptionSafetyLevel::kClear ? 0.25F : 0.10F;
    status.color.a = 1.0F;
    char text[256];
    if (decision.target_id == 0U) {
      std::snprintf(text, sizeof(text), "Perception safety: CLEAR");
    } else {
      std::snprintf(
          text, sizeof(text), "Safety #%u %s %.2fm%s%s",
          decision.target_id, PerceptionSafetyLevelName(decision.level),
          decision.distance_m, decision.zone_id.empty() ? "" : " zone=",
          decision.zone_id.c_str());
    }
    status.text = text;
    status.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_sec_);
    marker_pub_->publish(status);
  }

  void PublishInspectionRequired(
      const ManagedTarget& target,
      const builtin_interfaces::msg::Time& update_stamp) {
    robot_interfaces_perception::msg::PerceptionEvent event;
    event.header.stamp = update_stamp;
    event.header.frame_id = target_frame_;
    event.target_id = target.target_id;
    event.event_type = robot_interfaces_perception::msg::PerceptionEvent::INSPECTION_REQUIRED;
    event.class_name = target.class_name;
    event.target_pose.header = event.header;
    event.target_pose.pose.position.x = target.filtered_position.x;
    event.target_pose.pose.position.y = target.filtered_position.y;
    event.target_pose.pose.position.z = target.filtered_position.z;
    event.target_pose.pose.orientation.w = 1.0;
    event.confidence = static_cast<float>(target.confidence);
    event.severity = robot_interfaces_perception::msg::PerceptionEvent::SEVERITY_INFO;
    event_pub_->publish(event);
    RCLCPP_INFO(
        get_logger(),
        "inspection event emitted once: target=%u class=%s confidence=%.3f map=(%.3f, %.3f, %.3f)",
        target.target_id, target.class_name.c_str(), target.confidence,
        target.filtered_position.x, target.filtered_position.y,
        target.filtered_position.z);
  }

  void HandlePerceptionEvent(
      const robot_interfaces_perception::msg::PerceptionEvent& event) {
    if (event.event_type !=
        robot_interfaces_perception::msg::PerceptionEvent::INSPECTION_COMPLETED) {
      return;
    }
    auto timestamp = rclcpp::Time(event.header.stamp).nanoseconds();
    const auto target = std::find_if(
        target_manager_->targets().begin(), target_manager_->targets().end(),
        [&event](const ManagedTarget& candidate) {
          return candidate.target_id == event.target_id;
        });
    if (target != target_manager_->targets().end()) {
      timestamp = std::max(timestamp, target->last_observation_ns);
    }
    if (!target_manager_->MarkProcessed(event.target_id, timestamp, true)) {
      RCLCPP_WARN(
          get_logger(), "could not mark inspection target %u PROCESSED",
          event.target_id);
      return;
    }
    RCLCPP_INFO(
        get_logger(), "inspection target %u marked PROCESSED after task completion",
        event.target_id);
  }

  visualization_msgs::msg::Marker BuildManagedPointMarker(
      const robot_interfaces_perception::msg::DetectedObject3D& object) const {
    visualization_msgs::msg::Marker marker;
    marker.header = object.header;
    marker.ns = marker_namespace_;
    marker.id = static_cast<std::int32_t>(object.target_id * 2U);
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = object.position;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker_scale_;
    marker.scale.y = marker_scale_;
    marker.scale.z = marker_scale_;
    marker.color.a = 1.0F;
    switch (static_cast<TrackingState>(object.tracking_state)) {
      case TrackingState::kTentative:
        marker.color.r = 1.0F;
        marker.color.g = 0.75F;
        marker.color.b = 0.10F;
        break;
      case TrackingState::kConfirmed:
        marker.color.r = 0.10F;
        marker.color.g = 0.95F;
        marker.color.b = 0.35F;
        break;
      case TrackingState::kLost:
        marker.color.r = 0.65F;
        marker.color.g = 0.65F;
        marker.color.b = 0.65F;
        marker.color.a = 0.65F;
        break;
      case TrackingState::kProcessed:
        marker.color.r = 0.20F;
        marker.color.g = 0.60F;
        marker.color.b = 1.0F;
        break;
    }
    marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_sec_);
    return marker;
  }

  visualization_msgs::msg::Marker BuildManagedTextMarker(
      const robot_interfaces_perception::msg::DetectedObject3D& object) const {
    auto marker = BuildManagedPointMarker(object);
    marker.id = static_cast<std::int32_t>(object.target_id * 2U + 1U);
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.pose.position.z += marker_scale_ * 1.5;
    marker.scale.x = 0.0;
    marker.scale.y = 0.0;
    marker.scale.z = marker_scale_;
    marker.color.r = 1.0F;
    marker.color.g = 1.0F;
    marker.color.b = 1.0F;
    marker.color.a = 1.0F;
    char text[224];
    std::snprintf(
        text, sizeof(text),
        "#%u %s %s confidence=%.2f depth=%s map=(%.2f, %.2f, %.2f)",
        object.target_id, object.class_name.c_str(),
        TrackingStateName(static_cast<TrackingState>(object.tracking_state)),
        object.confidence, object.depth_valid ? "valid" : "invalid",
        object.position.x, object.position.y, object.position.z);
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
  std::string objects_3d_topic_;
  std::string event_topic_;
  std::string detection_topic_;
  std::string geometry_input_mode_;
  std::string marker_namespace_;
  std::string safety_event_topic_;
  std::string safety_robot_frame_;
  std::string safety_map_name_;
  std::string safety_zones_file_;
  BoundingBox2D bbox_;
  int sync_queue_size_ = 10;
  double sync_slop_sec_ = 0.05;
  double tf_timeout_sec_ = 0.1;
  double marker_scale_ = 0.18;
  double marker_lifetime_sec_ = 0.75;
  bool ground_truth_enabled_ = false;
  bool safety_danger_zone_enabled_ = true;
  double ground_truth_x_ = 0.0;
  double ground_truth_y_ = 0.0;
  double ground_truth_z_ = 0.0;
  std::size_t last_sample_count_ = 0U;
  std::size_t target_manager_update_count_ = 0U;
  double target_manager_latency_total_us_ = 0.0;

  std::unique_ptr<DepthProjector> projector_;
  std::unique_ptr<TargetManager> target_manager_;
  std::unique_ptr<InspectionEventPolicy> inspection_event_policy_;
  std::unique_ptr<PerceptionSafetyPolicy> safety_policy_;
  std::vector<robot_navigation::MapZone> danger_zones_;
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
  rclcpp::Publisher<
      robot_interfaces_perception::msg::DetectedObject3D>::SharedPtr objects_3d_pub_;
  rclcpp::Publisher<
      robot_interfaces_perception::msg::PerceptionEvent>::SharedPtr event_pub_;
  rclcpp::Publisher<
      robot_interfaces_perception::msg::PerceptionSafetyEvent>::SharedPtr
      safety_event_pub_;
  rclcpp::Subscription<
      robot_interfaces_perception::msg::PerceptionEvent>::SharedPtr event_sub_;
};

}  // namespace robot_perception

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_perception::GeometryValidationNode>());
  rclcpp::shutdown();
  return 0;
}
