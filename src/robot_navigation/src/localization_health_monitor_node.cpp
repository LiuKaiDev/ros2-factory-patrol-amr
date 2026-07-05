#include <algorithm>
#include <chrono>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/localization_health.hpp"
#include "robot_navigation/localization_health.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

namespace {

bool IsLocalizedState(const robot_navigation::LocalizationHealthState state) {
  return state == robot_navigation::LocalizationHealthState::kOk ||
         state == robot_navigation::LocalizationHealthState::kRecovered;
}

}  // namespace

class LocalizationHealthMonitorNode final : public rclcpp::Node {
 public:
  LocalizationHealthMonitorNode() : Node("localization_health_monitor_node") {
    amcl_pose_topic_ = declare_parameter<std::string>("amcl_pose_topic", "/amcl_pose");
    initialpose_topic_ = declare_parameter<std::string>("initialpose_topic", "/initialpose");
    health_topic_ = declare_parameter<std::string>("health_topic", "/localization/health");
    legacy_health_topic_ =
        declare_parameter<std::string>("legacy_health_topic", "/localization_health");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    amcl_timeout_sec_ = declare_parameter<double>("amcl_timeout_sec", 1.0);
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.5);
    lost_hold_time_sec_ = declare_parameter<double>("lost_hold_time_sec", 2.0);
    recovered_hold_time_sec_ = declare_parameter<double>("recovered_hold_time_sec", 1.0);
    publish_period_ms_ = declare_parameter<int>("publish_period_ms", 200);
    thresholds_.covariance_warn_xy =
        declare_parameter<double>("covariance_warn_xy", thresholds_.covariance_warn_xy);
    thresholds_.covariance_lost_xy =
        declare_parameter<double>("covariance_lost_xy", thresholds_.covariance_lost_xy);
    thresholds_.covariance_warn_yaw =
        declare_parameter<double>("covariance_warn_yaw", thresholds_.covariance_warn_yaw);
    thresholds_.covariance_lost_yaw =
        declare_parameter<double>("covariance_lost_yaw", thresholds_.covariance_lost_yaw);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    amcl_pose_sub_ =
        create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            amcl_pose_topic_, 10,
            [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
              HandleAmclPose(*msg);
            });
    initialpose_sub_ =
        create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            initialpose_topic_, 10,
            [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
              HandleInitialPose(*msg);
            });
    health_pub_ = create_publisher<std_msgs::msg::String>(health_topic_, 10);
    legacy_health_pub_ =
        create_publisher<robot_interfaces::msg::LocalizationHealth>(legacy_health_topic_, 10);

    const auto period = std::chrono::milliseconds(std::max(20, publish_period_ms_));
    timer_ = create_wall_timer(period, [this]() { Tick(); });
  }

 private:
  void HandleAmclPose(const geometry_msgs::msg::PoseWithCovarianceStamped& msg) {
    has_amcl_pose_ = true;
    last_amcl_stamp_ = now();
    covariance_ = robot_navigation::ExtractLocalizationCovariance(msg.pose.covariance);
  }

  void HandleInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped&) {
    if (state_ == robot_navigation::LocalizationHealthState::kLost ||
        state_ == robot_navigation::LocalizationHealthState::kRecovering) {
      recovering_requested_ = true;
      SetState(robot_navigation::LocalizationHealthState::kRecovering, "initialpose received");
    }
  }

  bool CanTransform(const std::string& target, const std::string& source) {
    std::string error;
    return tf_buffer_->canTransform(
        target, source, tf2::TimePointZero, tf2::durationFromSec(0.02), &error);
  }

  bool TfReady() {
    const bool map_to_odom = CanTransform(map_frame_, odom_frame_);
    const bool odom_to_base = CanTransform(odom_frame_, base_frame_);
    return map_to_odom && odom_to_base;
  }

  void Tick() {
    const auto stamp = now();
    const bool tf_ready = TfReady();
    UpdateIssueTimers(stamp, tf_ready);

    const double amcl_age = has_amcl_pose_ ? (stamp - last_amcl_stamp_).seconds() : 0.0;
    const bool amcl_timed_out =
        robot_navigation::IsTimedOut(has_amcl_pose_, amcl_age, amcl_timeout_sec_);
    const auto covariance_level =
        robot_navigation::EvaluateCovariance(covariance_, thresholds_);

    const bool covariance_lost_held =
        covariance_level == robot_navigation::LocalizationSignalLevel::kLost &&
        covariance_issue_active_ &&
        (stamp - covariance_issue_stamp_).seconds() >= lost_hold_time_sec_;
    const bool tf_lost_held = !tf_ready && tf_issue_active_ &&
                              (stamp - tf_issue_stamp_).seconds() >= tf_timeout_sec_;
    const bool stable =
        has_amcl_pose_ && !amcl_timed_out &&
        covariance_level == robot_navigation::LocalizationSignalLevel::kOk && tf_ready;

    if (!has_amcl_pose_) {
      SetState(robot_navigation::LocalizationHealthState::kUnknown, "waiting for amcl pose");
    } else if (recovering_requested_ && !stable) {
      SetState(
          robot_navigation::LocalizationHealthState::kRecovering,
          "waiting for AMCL convergence");
    } else if (stable) {
      HandleStable(stamp);
    } else if (amcl_timed_out) {
      recovering_requested_ = false;
      SetState(robot_navigation::LocalizationHealthState::kLost, "amcl pose timeout");
    } else if (covariance_lost_held) {
      recovering_requested_ = false;
      SetState(robot_navigation::LocalizationHealthState::kLost, "covariance above lost threshold");
    } else if (tf_lost_held) {
      recovering_requested_ = false;
      SetState(robot_navigation::LocalizationHealthState::kLost, "required TF unavailable");
    } else {
      SetState(
          robot_navigation::LocalizationHealthState::kUnstable,
          BuildUnstableReason(covariance_level, tf_ready));
    }

    PublishHealth(stamp, amcl_age, tf_ready);
  }

  void UpdateIssueTimers(const rclcpp::Time& stamp, const bool tf_ready) {
    const auto covariance_level =
        robot_navigation::EvaluateCovariance(covariance_, thresholds_);
    if (has_amcl_pose_ &&
        covariance_level == robot_navigation::LocalizationSignalLevel::kLost) {
      if (!covariance_issue_active_) {
        covariance_issue_stamp_ = stamp;
        covariance_issue_active_ = true;
      }
    } else {
      covariance_issue_active_ = false;
    }

    if (!tf_ready) {
      if (!tf_issue_active_) {
        tf_issue_stamp_ = stamp;
        tf_issue_active_ = true;
      }
    } else {
      tf_issue_active_ = false;
    }
  }

  void HandleStable(const rclcpp::Time& stamp) {
    const bool recovered_transition =
        state_ == robot_navigation::LocalizationHealthState::kLost ||
        state_ == robot_navigation::LocalizationHealthState::kRecovering ||
        recovering_requested_;
    if (recovered_transition) {
      recovering_requested_ = false;
      recovered_stamp_ = stamp;
      recovered_hold_active_ = true;
      SetState(robot_navigation::LocalizationHealthState::kRecovered, "localization recovered");
      return;
    }
    if (recovered_hold_active_ &&
        (stamp - recovered_stamp_).seconds() < recovered_hold_time_sec_) {
      SetState(robot_navigation::LocalizationHealthState::kRecovered, "localization recovered");
      return;
    }
    recovered_hold_active_ = false;
    SetState(robot_navigation::LocalizationHealthState::kOk, "localization stable");
  }

  std::string BuildUnstableReason(
      const robot_navigation::LocalizationSignalLevel covariance_level, const bool tf_ready) const {
    if (!tf_ready) {
      return "required TF temporarily unavailable";
    }
    if (covariance_level == robot_navigation::LocalizationSignalLevel::kLost) {
      return "covariance above lost threshold, waiting hold time";
    }
    return "covariance above warn threshold";
  }

  void SetState(
      const robot_navigation::LocalizationHealthState state, const std::string& reason) {
    if (state_ != state) {
      RCLCPP_INFO(
          get_logger(), "localization health: %s -> %s (%s)",
          robot_navigation::ToString(state_).c_str(),
          robot_navigation::ToString(state).c_str(), reason.c_str());
    }
    state_ = state;
    reason_ = reason;
  }

  void PublishHealth(const rclcpp::Time& stamp, const double amcl_age, const bool tf_ready) {
    const std::string state_text = robot_navigation::ToString(state_);
    const double covariance_xy = robot_navigation::MaxPlanarCovariance(covariance_);

    std_msgs::msg::String text;
    std::ostringstream out;
    out << state_text << " localized=" << (IsLocalizedState(state_) ? "true" : "false")
        << " covariance_xy=" << covariance_xy
        << " covariance_yaw=" << covariance_.yaw
        << " amcl_age_sec=" << amcl_age
        << " tf_ready=" << (tf_ready ? "true" : "false")
        << " reason=\"" << reason_ << "\"";
    text.data = out.str();
    health_pub_->publish(text);

    robot_interfaces::msg::LocalizationHealth legacy;
    legacy.header.stamp = stamp;
    legacy.localized = IsLocalizedState(state_);
    legacy.state = state_text;
    legacy.covariance_xy = covariance_xy;
    legacy.covariance_threshold = thresholds_.covariance_lost_xy;
    legacy.station_id = "";
    legacy.message = reason_;
    legacy_health_pub_->publish(legacy);
  }

  std::string amcl_pose_topic_;
  std::string initialpose_topic_;
  std::string health_topic_;
  std::string legacy_health_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  double amcl_timeout_sec_ = 1.0;
  double tf_timeout_sec_ = 0.5;
  double lost_hold_time_sec_ = 2.0;
  double recovered_hold_time_sec_ = 1.0;
  int publish_period_ms_ = 200;
  robot_navigation::LocalizationHealthThresholds thresholds_;
  robot_navigation::LocalizationCovariance covariance_;
  robot_navigation::LocalizationHealthState state_ =
      robot_navigation::LocalizationHealthState::kUnknown;
  std::string reason_ = "waiting for amcl pose";
  bool has_amcl_pose_ = false;
  bool recovering_requested_ = false;
  bool covariance_issue_active_ = false;
  bool tf_issue_active_ = false;
  bool recovered_hold_active_ = false;
  rclcpp::Time last_amcl_stamp_;
  rclcpp::Time covariance_issue_stamp_;
  rclcpp::Time tf_issue_stamp_;
  rclcpp::Time recovered_stamp_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr health_pub_;
  rclcpp::Publisher<robot_interfaces::msg::LocalizationHealth>::SharedPtr legacy_health_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationHealthMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
