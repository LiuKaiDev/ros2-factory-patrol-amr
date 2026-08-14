#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/safety_state.hpp"
#include "robot_interfaces_core/srv/set_control_mode.hpp"
#include "robot_interfaces_perception/msg/perception_safety_event.hpp"
#include "robot_teleop/cmd_vel_safety.hpp"
#include "robot_teleop/perception_safety_input.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

namespace {

geometry_msgs::msg::Twist ZeroTwist() {
  return robot_teleop::MakeZeroTwist();
}

}  // namespace

class CmdVelMuxNode final : public rclcpp::Node {
 public:
  CmdVelMuxNode() : Node("cmd_vel_mux_node") {
    active_source_ = declare_parameter<std::string>("default_source", "teleop");
    watchdog_timeout_ms_ = declare_parameter<int>("watchdog_timeout_ms", 500);
    dynamic_angular_limit_radps_ = declare_parameter<double>("dynamic_angular_limit_radps", 0.6);
    perception_safety_enabled_ =
        declare_parameter<bool>("perception_safety_enabled", true);
    perception_safety_topic_ = declare_parameter<std::string>(
        "perception_safety_topic", "/perception/safety_event");
    perception_event_timeout_sec_ =
        declare_parameter<double>("perception_event_timeout_sec", 1.5);
    perception_speed_limit_linear_mps_ =
        declare_parameter<double>("perception_speed_limit_linear_mps", 0.15);
    perception_speed_limit_angular_radps_ =
        declare_parameter<double>("perception_speed_limit_angular_radps", 0.4);
    safety_state_topic_ =
        declare_parameter<std::string>("safety_state_topic", "/safety/state");
    safety_reason_topic_ =
        declare_parameter<std::string>("safety_reason_topic", "/safety/reason");
    if (perception_safety_topic_.empty() || safety_state_topic_.empty() ||
        safety_reason_topic_.empty() || !std::isfinite(perception_event_timeout_sec_) ||
        perception_event_timeout_sec_ <= 0.0 ||
        !std::isfinite(perception_speed_limit_linear_mps_) ||
        perception_speed_limit_linear_mps_ <= 0.0 ||
        !std::isfinite(perception_speed_limit_angular_radps_) ||
        perception_speed_limit_angular_radps_ <= 0.0) {
      throw std::invalid_argument("perception safety mux parameters are invalid");
    }

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    active_pub_ = create_publisher<std_msgs::msg::String>("/cmd_vel_mux/active_source", 10);
    estop_state_pub_ = create_publisher<std_msgs::msg::Bool>("/emergency_stop/state", 10);
    takeover_state_pub_ = create_publisher<std_msgs::msg::Bool>("/cmd_vel_mux/manual_takeover", 10);
    safety_state_pub_ = create_publisher<std_msgs::msg::String>(safety_state_topic_, 10);
    safety_reason_pub_ = create_publisher<std_msgs::msg::String>(safety_reason_topic_, 10);

    AddInput("nav2", "/nav2_cmd_vel");
    AddInput("tracking", "/tracking_cmd_vel");
    AddInput("teleop", "/teleop_cmd_vel");
    takeover_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/manual_takeover/state", 10,
        [this](std_msgs::msg::Bool::SharedPtr msg) { manual_takeover_ = msg->data; });
    safety_sub_ = create_subscription<robot_interfaces::msg::SafetyState>(
        "/safety_state", 10, [this](robot_interfaces::msg::SafetyState::SharedPtr msg) {
          runtime_safety_stop_ = msg->safety_stop;
          dynamic_speed_limit_mps_ = msg->effective_speed_limit_mps;
        });
    perception_safety_sub_ = create_subscription<
        robot_interfaces_perception::msg::PerceptionSafetyEvent>(
        perception_safety_topic_, rclcpp::QoS(10).reliable(),
        [this](
            const robot_interfaces_perception::msg::PerceptionSafetyEvent::SharedPtr msg) {
          if (!perception_safety_enabled_) {
            return;
          }
          if (!robot_teleop::UpdatePerceptionSafetySnapshot(
                  &perception_safety_, msg->safety_state, msg->target_id,
                  msg->class_name, msg->event_type, msg->distance_m, msg->source,
                  msg->reason, rclcpp::Time(msg->header.stamp).nanoseconds(),
                  now().nanoseconds(), perception_event_timeout_sec_)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "rejected malformed, stale, future, or out-of-order perception safety event");
          }
        });

    v2_set_source_srv_ = create_service<robot_interfaces_core::srv::SetControlMode>(
        "/v2/set_cmd_source",
        [this](const std::shared_ptr<robot_interfaces_core::srv::SetControlMode::Request> request,
               std::shared_ptr<robot_interfaces_core::srv::SetControlMode::Response> response) {
          SetSource(request, response);
        });
    enable_estop_srv_ = create_service<std_srvs::srv::SetBool>(
        "/enable_emergency_stop",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
          SetEmergencyStop(request, response, true);
        });
    clear_estop_srv_ = create_service<std_srvs::srv::SetBool>(
        "/clear_emergency_stop",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
          SetEmergencyStop(request, response, false);
        });

    timer_ = create_wall_timer(50ms, [this]() { Tick(); });
  }

 private:
  struct InputState {
    geometry_msgs::msg::Twist last_twist;
    rclcpp::Time last_stamp;
    bool received = false;
  };

  void AddInput(const std::string& name, const std::string& topic) {
    inputs_[name] = InputState{ZeroTwist(), now(), false};
    subscriptions_.push_back(create_subscription<geometry_msgs::msg::Twist>(
        topic, 10, [this, name](geometry_msgs::msg::Twist::SharedPtr msg) {
          inputs_[name].last_twist = *msg;
          inputs_[name].last_stamp = now();
          inputs_[name].received = true;
        }));
  }

  template <typename RequestT, typename ResponseT>
  void SetSource(const std::shared_ptr<RequestT>& request, const std::shared_ptr<ResponseT>& response) {
    if (inputs_.find(request->mode) == inputs_.end()) {
      response->success = false;
      response->message = "unknown cmd source: " + request->mode;
      return;
    }
    active_source_ = request->mode;
    response->success = true;
    response->message = "active cmd source set to " + active_source_;
  }

  void SetEmergencyStop(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response, const bool target_state) {
    if (target_state) {
      emergency_stop_ = request->data;
    } else if (request->data) {
      emergency_stop_ = false;
    }
    response->success = true;
    response->message = emergency_stop_ ? "emergency stop active" : "emergency stop cleared";
  }

  bool WatchdogExpired(const InputState& input) const {
    if (!input.received) {
      return true;
    }
    const auto age_ms = (now() - input.last_stamp).nanoseconds() / 1000000;
    return age_ms > watchdog_timeout_ms_;
  }

  void Tick() {
    const auto perception_state = perception_safety_enabled_
                                      ? robot_teleop::PerceptionSafetyContribution(
                                            perception_safety_, now().nanoseconds(),
                                            perception_event_timeout_sec_)
                                      : robot_teleop::SafetyState::kNormal;
    const bool perception_stop =
        perception_state == robot_teleop::SafetyState::kPerceptionStop;
    const bool perception_limited =
        perception_state == robot_teleop::SafetyState::kSpeedLimited;
    std_msgs::msg::String active_msg;
    active_msg.data = robot_teleop::DescribeCommandState(
        emergency_stop_, runtime_safety_stop_ || perception_stop,
        manual_takeover_, active_source_);
    active_pub_->publish(active_msg);
    std_msgs::msg::Bool estop_msg;
    estop_msg.data = emergency_stop_;
    estop_state_pub_->publish(estop_msg);
    std_msgs::msg::Bool takeover_msg;
    takeover_msg.data = manual_takeover_;
    takeover_state_pub_->publish(takeover_msg);

    const std::string selected_source =
        robot_teleop::SelectCommandSource(manual_takeover_, active_source_);
    const auto input_it = inputs_.find(selected_source);
    const bool input_available = input_it != inputs_.end();
    const bool input_fresh = input_available && !WatchdogExpired(input_it->second);

    std::vector<robot_teleop::SafetyState> states{
        robot_teleop::SafetyState::kNormal, perception_state};
    std::vector<std::string> reasons;
    if (emergency_stop_) {
      states.push_back(robot_teleop::SafetyState::kEmergencyStop);
      reasons.push_back("emergency stop active");
    }
    if (runtime_safety_stop_) {
      states.push_back(robot_teleop::SafetyState::kChassisFault);
      reasons.push_back("legacy /safety_state requested safety_stop");
    }
    if (dynamic_speed_limit_mps_ > 0.0) {
      states.push_back(robot_teleop::SafetyState::kSpeedLimited);
      reasons.push_back("runtime dynamic speed limit active");
    }
    if (!input_fresh) {
      states.push_back(robot_teleop::SafetyState::kCommunicationLost);
      reasons.push_back("selected cmd_vel source stale or missing: " + selected_source);
    }
    if (perception_state != robot_teleop::SafetyState::kNormal) {
      reasons.push_back(perception_safety_.reason);
    } else if (perception_safety_enabled_ && perception_safety_.received &&
               !robot_teleop::PerceptionSafetyInputIsFresh(
                   perception_safety_, now().nanoseconds(),
                   perception_event_timeout_sec_)) {
      reasons.push_back("perception safety event stale; perception restriction removed");
    }
    PublishSafetyState(robot_teleop::ResolveHighestPriority(states), reasons);

    robot_teleop::CmdVelSafetyGateRequest gate_request;
    gate_request.input_twist = input_available ? input_it->second.last_twist : ZeroTwist();
    gate_request.emergency_stop = emergency_stop_;
    gate_request.runtime_safety_stop = runtime_safety_stop_ || perception_stop;
    gate_request.input_available = input_available;
    gate_request.input_fresh = input_fresh;
    gate_request.dynamic_speed_limit_mps = EffectiveLinearLimit(perception_limited);
    gate_request.dynamic_angular_limit_radps = EffectiveAngularLimit(perception_limited);
    const auto gate_decision = robot_teleop::EvaluateCmdVelSafetyGate(gate_request);

    if (gate_decision.stop_reason == robot_teleop::CmdVelStopReason::kInputUnavailable) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "cmd_vel source '%s' is not registered; publishing zero twist",
                           selected_source.c_str());
    } else if (gate_decision.stop_reason == robot_teleop::CmdVelStopReason::kInputStale) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "cmd_vel source '%s' watchdog expired; publishing zero twist",
                           selected_source.c_str());
    }
    cmd_pub_->publish(gate_decision.output_twist);
  }

  double EffectiveLinearLimit(const bool perception_limited) const {
    if (!perception_limited) {
      return dynamic_speed_limit_mps_;
    }
    if (dynamic_speed_limit_mps_ <= 0.0) {
      return perception_speed_limit_linear_mps_;
    }
    return std::min(
        dynamic_speed_limit_mps_, perception_speed_limit_linear_mps_);
  }

  double EffectiveAngularLimit(const bool perception_limited) const {
    if (!perception_limited) {
      return dynamic_angular_limit_radps_;
    }
    if (dynamic_angular_limit_radps_ <= 0.0) {
      return perception_speed_limit_angular_radps_;
    }
    return std::min(
        dynamic_angular_limit_radps_, perception_speed_limit_angular_radps_);
  }

  void PublishSafetyState(
      const robot_teleop::SafetyState state,
      const std::vector<std::string>& reasons) {
    std_msgs::msg::String state_msg;
    state_msg.data = robot_teleop::ToString(state);
    safety_state_pub_->publish(state_msg);

    std::ostringstream reason;
    if (reasons.empty()) {
      reason << "all monitored safety inputs nominal";
    } else {
      for (std::size_t index = 0U; index < reasons.size(); ++index) {
        if (index > 0U) {
          reason << "; ";
        }
        reason << reasons[index];
      }
    }
    std_msgs::msg::String reason_msg;
    reason_msg.data = reason.str();
    safety_reason_pub_->publish(reason_msg);
  }

  std::string active_source_;
  std::string perception_safety_topic_;
  std::string safety_state_topic_;
  std::string safety_reason_topic_;
  int watchdog_timeout_ms_;
  bool emergency_stop_ = false;
  bool manual_takeover_ = false;
  bool runtime_safety_stop_ = false;
  bool perception_safety_enabled_ = true;
  double dynamic_speed_limit_mps_ = 0.0;
  double dynamic_angular_limit_radps_ = 0.6;
  double perception_event_timeout_sec_ = 1.5;
  double perception_speed_limit_linear_mps_ = 0.15;
  double perception_speed_limit_angular_radps_ = 0.4;
  robot_teleop::PerceptionSafetySnapshot perception_safety_;
  std::unordered_map<std::string, InputState> inputs_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr> subscriptions_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr takeover_sub_;
  rclcpp::Subscription<robot_interfaces::msg::SafetyState>::SharedPtr safety_sub_;
  rclcpp::Subscription<
      robot_interfaces_perception::msg::PerceptionSafetyEvent>::SharedPtr
      perception_safety_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr takeover_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr safety_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr safety_reason_pub_;
  rclcpp::Service<robot_interfaces_core::srv::SetControlMode>::SharedPtr v2_set_source_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_estop_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr clear_estop_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelMuxNode>());
  rclcpp::shutdown();
  return 0;
}
