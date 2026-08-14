#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/action/navigate_sequence.hpp"
#include "robot_interfaces_perception/msg/perception_event.hpp"
#include "robot_tasks/visual_inspection_mission.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;

namespace robot_tasks {

class VisualInspectionTaskNode final : public rclcpp::Node {
 public:
  using NavigateSequence = robot_interfaces::action::NavigateSequence;
  using GoalHandleNavigateSequence =
      rclcpp_action::ClientGoalHandle<NavigateSequence>;

  VisualInspectionTaskNode() : Node("visual_inspection_task_node") {
    VisualInspectionConfig config;
    config.enabled = declare_parameter<bool>("inspection.enabled", true);
    config.allowed_classes = declare_parameter<std::vector<std::string>>(
        "inspection.allowed_classes", {"chair"});
    config.min_confidence =
        declare_parameter<double>("inspection.min_confidence", 0.5);
    config.standoff_distance =
        declare_parameter<double>("inspection.standoff_distance", 1.2);
    config.processed_on_success =
        declare_parameter<bool>("inspection.processed_on_success", true);
    mission_ = std::make_unique<VisualInspectionMission>(config);

    event_topic_ = declare_parameter<std::string>(
        "inspection.event_topic", "/perception/events");
    action_name_ = declare_parameter<std::string>(
        "inspection.navigation_action", "/navigate_sequence");
    observation_pose_topic_ = declare_parameter<std::string>(
        "inspection.observation_pose_topic", "/inspection/observation_pose");
    status_topic_ = declare_parameter<std::string>(
        "inspection.status_topic", "/inspection/status");
    marker_topic_ = declare_parameter<std::string>(
        "inspection.marker_topic", "/inspection/observation_marker");
    map_frame_ = declare_parameter<std::string>("inspection.map_frame", "map");
    robot_frame_ =
        declare_parameter<std::string>("inspection.robot_frame", "base_link");
    action_server_timeout_ms_ = declare_parameter<int>(
        "inspection.action_server_timeout_ms", 3000);
    tf_wait_timeout_sec_ =
        declare_parameter<double>("inspection.tf_wait_timeout_sec", 5.0);
    retry_count_ = declare_parameter<int>("inspection.retry_count", 0);
    retry_delay_sec_ =
        declare_parameter<double>("inspection.retry_delay_sec", 2.0);
    if (event_topic_.empty() || action_name_.empty() || map_frame_ != "map" ||
        robot_frame_.empty() || action_server_timeout_ms_ < 0 ||
        !std::isfinite(tf_wait_timeout_sec_) || tf_wait_timeout_sec_ <= 0.0 ||
        !std::isfinite(retry_delay_sec_) ||
        retry_count_ < 0 || retry_delay_sec_ <= 0.0) {
      throw std::invalid_argument("visual inspection ROS parameters are invalid");
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    action_client_ =
        rclcpp_action::create_client<NavigateSequence>(this, action_name_);
    const auto event_qos = rclcpp::QoS(10).reliable().transient_local();
    event_pub_ = create_publisher<
        robot_interfaces_perception::msg::PerceptionEvent>(event_topic_, event_qos);
    event_sub_ = create_subscription<
        robot_interfaces_perception::msg::PerceptionEvent>(
        event_topic_, event_qos,
        [this](const robot_interfaces_perception::msg::PerceptionEvent::SharedPtr event) {
          HandleEvent(*event);
        });
    observation_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        observation_pose_topic_, rclcpp::QoS(1).transient_local());
    status_pub_ = create_publisher<std_msgs::msg::String>(
        status_topic_, rclcpp::QoS(10).reliable().transient_local());
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
        marker_topic_, rclcpp::QoS(1).transient_local());
    tf_wait_timer_ = create_wall_timer(100ms, [this]() { TryPendingEvent(); });
    PublishStatus("IDLE", "visual inspection task ready");
    RCLCPP_INFO(
        get_logger(),
        "visual inspection task ready: events=%s navigation=%s robot_tasks owns Nav2 interaction",
        event_topic_.c_str(), action_name_.c_str());
  }

 private:
  void HandleEvent(
      const robot_interfaces_perception::msg::PerceptionEvent& event) {
    if (event.event_type !=
        robot_interfaces_perception::msg::PerceptionEvent::INSPECTION_REQUIRED) {
      return;
    }
    if (pending_event_) {
      return;
    }
    pending_event_ = event;
    tf_wait_deadline_ = std::chrono::steady_clock::now() +
                        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                            std::chrono::duration<double>(tf_wait_timeout_sec_));
    TryPendingEvent();
  }

  void TryPendingEvent() {
    if (!pending_event_) {
      return;
    }
    geometry_msgs::msg::PoseStamped robot_pose;
    try {
      const auto transform = tf_buffer_->lookupTransform(
          map_frame_, robot_frame_, tf2::TimePointZero,
          tf2::durationFromSec(0.0));
      robot_pose.header = transform.header;
      robot_pose.header.frame_id = map_frame_;
      robot_pose.pose.position.x = transform.transform.translation.x;
      robot_pose.pose.position.y = transform.transform.translation.y;
      robot_pose.pose.position.z = transform.transform.translation.z;
      robot_pose.pose.orientation = transform.transform.rotation;
    } catch (const tf2::TransformException& error) {
      if (std::chrono::steady_clock::now() < tf_wait_deadline_) {
        return;
      }
      PublishStatus("FAILED", "robot map pose unavailable");
      RCLCPP_WARN(
          get_logger(), "inspection target %u rejected: map <- %s TF failed: %s",
          pending_event_->target_id, robot_frame_.c_str(), error.what());
      pending_event_.reset();
      return;
    }

    const auto event = *pending_event_;
    pending_event_.reset();
    const auto decision = mission_->Request(event, robot_pose);
    if (!decision.accepted || !decision.observation_pose) {
      RCLCPP_INFO(
          get_logger(), "inspection event for target %u ignored: %s",
          event.target_id, decision.reason.c_str());
      return;
    }

    active_event_ = event;
    active_observation_pose_ = *decision.observation_pose;
    active_observation_pose_.header.stamp = now();
    attempts_ = 0;
    observation_pose_pub_->publish(active_observation_pose_);
    PublishObservationMarker(active_observation_pose_, event.target_id);
    PublishStatus("REQUESTED", "observation pose planned for target " +
                                   std::to_string(event.target_id));
    RCLCPP_INFO(
        get_logger(),
        "inspection requested: target=%u class=%s target=(%.3f, %.3f) observation=(%.3f, %.3f)",
        event.target_id, event.class_name.c_str(),
        event.target_pose.pose.position.x, event.target_pose.pose.position.y,
        active_observation_pose_.pose.position.x,
        active_observation_pose_.pose.position.y);
    SendActiveGoal();
  }

  void SendActiveGoal() {
    if (mission_->active_target_id() == 0U) {
      return;
    }
    ++attempts_;
    if (!action_client_->wait_for_action_server(
            std::chrono::milliseconds(action_server_timeout_ms_))) {
      HandleAttemptFailure("navigate sequence action server unavailable");
      return;
    }

    NavigateSequence::Goal goal;
    goal.loop = false;
    active_observation_pose_.header.stamp = now();
    goal.goals.push_back(active_observation_pose_);
    auto options = rclcpp_action::Client<NavigateSequence>::SendGoalOptions();
    options.goal_response_callback =
        [this](const GoalHandleNavigateSequence::SharedPtr goal_handle) {
          if (!goal_handle) {
            HandleAttemptFailure("navigate sequence rejected inspection goal");
            return;
          }
          active_goal_handle_ = goal_handle;
          mission_->MarkNavigating();
          PublishStatus("NAVIGATING", "inspection goal accepted");
          RCLCPP_INFO(
              get_logger(), "Nav2 adapter accepted inspection target %u",
              active_event_.target_id);
        };
    options.feedback_callback =
        [this](GoalHandleNavigateSequence::SharedPtr,
               const std::shared_ptr<const NavigateSequence::Feedback> feedback) {
          PublishStatus("NAVIGATING", feedback->state + ": " + feedback->message);
        };
    options.result_callback =
        [this](const GoalHandleNavigateSequence::WrappedResult& result) {
          active_goal_handle_.reset();
          const bool success =
              result.code == rclcpp_action::ResultCode::SUCCEEDED &&
              result.result && result.result->success;
          if (!success) {
            HandleAttemptFailure(
                result.result ? result.result->message
                              : "navigate sequence returned no result");
            return;
          }
          const auto outcome = mission_->Finish(true);
          PublishStatus("SUCCEEDED", "inspection navigation completed");
          RCLCPP_INFO(
              get_logger(), "inspection target %u navigation succeeded",
              outcome.target_id);
          if (outcome.publish_completion) {
            PublishCompletion(outcome.target_id);
          }
        };
    action_client_->async_send_goal(goal, options);
  }

  void HandleAttemptFailure(const std::string& reason) {
    RCLCPP_WARN(
        get_logger(), "inspection attempt %d for target %u failed: %s",
        attempts_, active_event_.target_id, reason.c_str());
    if (attempts_ <= retry_count_) {
      PublishStatus("REQUESTED", "inspection retry scheduled: " + reason);
      retry_timer_ = create_wall_timer(
          std::chrono::duration<double>(retry_delay_sec_), [this]() {
            retry_timer_->cancel();
            retry_timer_.reset();
            SendActiveGoal();
          });
      return;
    }
    const auto outcome = mission_->Finish(false);
    PublishStatus("FAILED", reason);
    RCLCPP_ERROR(
        get_logger(), "inspection target %u failed after %d attempt(s)",
        outcome.target_id, attempts_);
  }

  void PublishCompletion(const std::uint32_t target_id) {
    auto completion = active_event_;
    completion.header.stamp = now();
    completion.header.frame_id = map_frame_;
    completion.target_pose.header = completion.header;
    completion.event_type =
        robot_interfaces_perception::msg::PerceptionEvent::INSPECTION_COMPLETED;
    completion.severity =
        robot_interfaces_perception::msg::PerceptionEvent::SEVERITY_INFO;
    completion.target_id = target_id;
    event_pub_->publish(completion);
  }

  void PublishStatus(const std::string& state, const std::string& message) {
    std_msgs::msg::String status;
    status.data = state + ": " + message;
    status_pub_->publish(status);
  }

  void PublishObservationMarker(
      const geometry_msgs::msg::PoseStamped& pose,
      const std::uint32_t target_id) {
    visualization_msgs::msg::Marker marker;
    marker.header = pose.header;
    marker.ns = "visual_inspection_observation";
    marker.id = static_cast<std::int32_t>(target_id);
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = pose.pose;
    marker.scale.x = 0.55;
    marker.scale.y = 0.12;
    marker.scale.z = 0.12;
    marker.color.r = 1.0F;
    marker.color.g = 0.35F;
    marker.color.b = 0.05F;
    marker.color.a = 1.0F;
    marker_pub_->publish(marker);
  }

  std::string event_topic_;
  std::string action_name_;
  std::string observation_pose_topic_;
  std::string status_topic_;
  std::string marker_topic_;
  std::string map_frame_;
  std::string robot_frame_;
  int action_server_timeout_ms_ = 3000;
  int retry_count_ = 0;
  int attempts_ = 0;
  double tf_wait_timeout_sec_ = 5.0;
  double retry_delay_sec_ = 2.0;
  std::optional<robot_interfaces_perception::msg::PerceptionEvent> pending_event_;
  std::chrono::steady_clock::time_point tf_wait_deadline_;
  std::unique_ptr<VisualInspectionMission> mission_;
  robot_interfaces_perception::msg::PerceptionEvent active_event_;
  geometry_msgs::msg::PoseStamped active_observation_pose_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Client<NavigateSequence>::SharedPtr action_client_;
  GoalHandleNavigateSequence::SharedPtr active_goal_handle_;
  rclcpp::Publisher<robot_interfaces_perception::msg::PerceptionEvent>::SharedPtr
      event_pub_;
  rclcpp::Subscription<robot_interfaces_perception::msg::PerceptionEvent>::SharedPtr
      event_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      observation_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
  rclcpp::TimerBase::SharedPtr tf_wait_timer_;
};

}  // namespace robot_tasks

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_tasks::VisualInspectionTaskNode>());
  rclcpp::shutdown();
  return 0;
}
