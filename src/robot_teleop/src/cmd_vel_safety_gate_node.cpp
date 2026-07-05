#include <algorithm>
#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/chassis_state.hpp"
#include "robot_interfaces/msg/safety_state.hpp"
#include "robot_interfaces_core/srv/set_control_mode.hpp"
#include "robot_teleop/cmd_vel_safety.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

class CmdVelSafetyGateNode final : public rclcpp::Node {
 public:
  CmdVelSafetyGateNode() : Node("cmd_vel_safety_gate_node") {
    input_topic_ = declare_parameter<std::string>("input_topic", "/muxed_cmd_vel");
    active_source_hint_ = declare_parameter<std::string>("default_source", "twist_mux");
    watchdog_timeout_ms_ = declare_parameter<int>("watchdog_timeout_ms", 500);
    dynamic_angular_limit_radps_ =
        declare_parameter<double>("dynamic_angular_limit_radps", 0.6);
    localization_health_topic_ =
        declare_parameter<std::string>("localization_health_topic", "/localization/health");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    chassis_state_topic_ = declare_parameter<std::string>("chassis_state_topic", "/chassis/state");
    safety_state_topic_ = declare_parameter<std::string>("safety_state_topic", "/safety/state");
    safety_reason_topic_ =
        declare_parameter<std::string>("safety_reason_topic", "/safety/reason");
    scan_timeout_sec_ = declare_parameter<double>("scan_timeout_sec", 1.0);
    odom_timeout_sec_ = declare_parameter<double>("odom_timeout_sec", 1.0);
    localization_timeout_sec_ = declare_parameter<double>("localization_timeout_sec", 1.0);
    chassis_state_timeout_sec_ = declare_parameter<double>("chassis_state_timeout_sec", 1.0);
    safety_startup_grace_sec_ = declare_parameter<double>("safety_startup_grace_sec", 2.0);
    localization_lost_stop_ = declare_parameter<bool>("localization_lost_stop", true);
    sensor_stale_stop_ = declare_parameter<bool>("sensor_stale_stop", true);
    chassis_fault_stop_ = declare_parameter<bool>("chassis_fault_stop", true);
    communication_lost_stop_ = declare_parameter<bool>("communication_lost_stop", true);
    emergency_stop_requires_reset_ =
        declare_parameter<bool>("emergency_stop_requires_reset", true);
    speed_limited_max_linear_mps_ =
        declare_parameter<double>("speed_limited_max_linear_mps", 0.15);
    speed_limited_max_angular_radps_ =
        declare_parameter<double>("speed_limited_max_angular_radps", 0.4);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    active_pub_ = create_publisher<std_msgs::msg::String>("/cmd_vel_mux/active_source", 10);
    estop_state_pub_ = create_publisher<std_msgs::msg::Bool>("/emergency_stop/state", 10);
    takeover_state_pub_ = create_publisher<std_msgs::msg::Bool>("/cmd_vel_mux/manual_takeover", 10);
    takeover_lock_pub_ =
        create_publisher<std_msgs::msg::Bool>("/cmd_vel_mux/manual_takeover_lock", 10);
    safety_state_pub_ = create_publisher<std_msgs::msg::String>(safety_state_topic_, 10);
    safety_reason_pub_ = create_publisher<std_msgs::msg::String>(safety_reason_topic_, 10);

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        input_topic_, 10, [this](geometry_msgs::msg::Twist::SharedPtr msg) {
          last_twist_ = *msg;
          last_stamp_ = now();
          received_ = true;
        });
    takeover_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/manual_takeover/state", 10,
        [this](std_msgs::msg::Bool::SharedPtr msg) { manual_takeover_ = msg->data; });
    safety_sub_ = create_subscription<robot_interfaces::msg::SafetyState>(
        "/safety_state", 10, [this](robot_interfaces::msg::SafetyState::SharedPtr msg) {
          runtime_safety_stop_ = msg->safety_stop;
          dynamic_speed_limit_mps_ = msg->effective_speed_limit_mps;
        });
    localization_health_sub_ = create_subscription<std_msgs::msg::String>(
        localization_health_topic_, 10, [this](std_msgs::msg::String::SharedPtr msg) {
          localization_health_ = msg->data;
          localization_seen_ = true;
          localization_stamp_ = now();
        });
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, 10, [this](sensor_msgs::msg::LaserScan::SharedPtr) {
          scan_seen_ = true;
          scan_stamp_ = now();
        });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10, [this](nav_msgs::msg::Odometry::SharedPtr) {
          odom_seen_ = true;
          odom_stamp_ = now();
        });
    chassis_state_sub_ = create_subscription<robot_interfaces::msg::ChassisState>(
        chassis_state_topic_, 10,
        [this](robot_interfaces::msg::ChassisState::SharedPtr msg) {
          chassis_state_ = *msg;
          chassis_seen_ = true;
          chassis_stamp_ = now();
          if (robot_teleop::SafetyStateFromChassisStatus(msg->connected, msg->status) ==
              robot_teleop::SafetyState::kEmergencyStop) {
            emergency_stop_latched_ = true;
          }
        });

    v2_set_source_srv_ = create_service<robot_interfaces_core::srv::SetControlMode>(
        "/v2/set_cmd_source",
        [this](const std::shared_ptr<robot_interfaces_core::srv::SetControlMode::Request> request,
               std::shared_ptr<robot_interfaces_core::srv::SetControlMode::Response> response) {
          SetSourceHint(request, response);
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

    last_stamp_ = now();
    started_stamp_ = now();
    timer_ = create_wall_timer(50ms, [this]() { Tick(); });
  }

 private:
  template <typename RequestT, typename ResponseT>
  void SetSourceHint(const std::shared_ptr<RequestT>& request, const std::shared_ptr<ResponseT>& response) {
    if (request->mode != "nav2" && request->mode != "tracking" && request->mode != "teleop" &&
        request->mode != "twist_mux") {
      response->success = false;
      response->message = "unknown cmd source: " + request->mode;
      return;
    }
    active_source_hint_ = request->mode;
    response->success = true;
    response->message =
        "cmd source hint set to " + active_source_hint_ + "; twist_mux still arbitrates by priority";
  }

  void SetEmergencyStop(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response, const bool target_state) {
    if (target_state) {
      emergency_stop_ = request->data;
      if (request->data) {
        emergency_stop_latched_ = true;
      }
    } else if (request->data) {
      emergency_stop_ = false;
      emergency_stop_latched_ = false;
    }
    response->success = true;
    response->message =
        emergency_stop_latched_ ? "emergency stop active" : "emergency stop cleared";
  }

  bool WatchdogExpired() const {
    if (!received_) {
      return true;
    }
    const auto age_ms = (now() - last_stamp_).nanoseconds() / 1000000;
    return age_ms > watchdog_timeout_ms_;
  }

  bool InStartupGrace() const {
    return ((now() - started_stamp_).nanoseconds() / 1000000000.0) < safety_startup_grace_sec_;
  }

  bool IsFresh(const bool seen, const rclcpp::Time& stamp, const double timeout_sec) const {
    if (!seen) {
      return false;
    }
    return ((now() - stamp).nanoseconds() / 1000000000.0) <= timeout_sec;
  }

  std::string JoinReasons(const std::vector<std::string>& reasons) const {
    if (reasons.empty()) {
      return "all monitored safety inputs nominal";
    }
    std::ostringstream out;
    for (std::size_t i = 0; i < reasons.size(); ++i) {
      if (i > 0) {
        out << "; ";
      }
      out << reasons[i];
    }
    return out.str();
  }

  robot_teleop::SafetyState ResolveSafetyState(std::string* reason) const {
    std::vector<robot_teleop::SafetyState> states{robot_teleop::SafetyState::kNormal};
    std::vector<std::string> reasons;

    if (emergency_stop_ || emergency_stop_latched_) {
      states.push_back(robot_teleop::SafetyState::kEmergencyStop);
      reasons.push_back("emergency stop active");
    }
    if (runtime_safety_stop_) {
      states.push_back(robot_teleop::SafetyState::kChassisFault);
      reasons.push_back("legacy /safety_state requested safety_stop");
    }
    if (manual_takeover_) {
      states.push_back(robot_teleop::SafetyState::kManualTakeover);
      reasons.push_back("manual takeover active");
    }
    if (dynamic_speed_limit_mps_ > 0.0) {
      states.push_back(robot_teleop::SafetyState::kSpeedLimited);
      reasons.push_back("runtime dynamic speed limit active");
    }
    if (!InStartupGrace() && WatchdogExpired()) {
      states.push_back(robot_teleop::SafetyState::kCommunicationLost);
      reasons.push_back("muxed cmd_vel stale or missing: " + input_topic_);
    }

    if (!InStartupGrace()) {
      if (!IsFresh(scan_seen_, scan_stamp_, scan_timeout_sec_)) {
        states.push_back(robot_teleop::SafetyState::kSensorStale);
        reasons.push_back("scan topic stale or missing: " + scan_topic_);
      }
      if (!IsFresh(odom_seen_, odom_stamp_, odom_timeout_sec_)) {
        states.push_back(robot_teleop::SafetyState::kSensorStale);
        reasons.push_back("odom topic stale or missing: " + odom_topic_);
      }
      if (!IsFresh(localization_seen_, localization_stamp_, localization_timeout_sec_)) {
        states.push_back(robot_teleop::SafetyState::kLocalizationLost);
        reasons.push_back("localization health stale or missing: " + localization_health_topic_);
      }
      if (!IsFresh(chassis_seen_, chassis_stamp_, chassis_state_timeout_sec_)) {
        states.push_back(robot_teleop::SafetyState::kCommunicationLost);
        reasons.push_back("chassis state stale or missing: " + chassis_state_topic_);
      }
    }

    if (localization_seen_) {
      const auto localization_state =
          robot_teleop::SafetyStateFromLocalizationHealth(localization_health_);
      states.push_back(localization_state);
      if (localization_state == robot_teleop::SafetyState::kLocalizationLost) {
        reasons.push_back("localization reports LOCALIZATION_LOST");
      } else if (localization_state == robot_teleop::SafetyState::kSpeedLimited) {
        reasons.push_back("localization reports LOCALIZATION_UNSTABLE");
      } else if (localization_state == robot_teleop::SafetyState::kRecovery) {
        reasons.push_back("localization recovery in progress");
      }
    }

    if (chassis_seen_) {
      const auto chassis_state =
          robot_teleop::SafetyStateFromChassisStatus(chassis_state_.connected, chassis_state_.status);
      states.push_back(chassis_state);
      if (chassis_state != robot_teleop::SafetyState::kNormal) {
        reasons.push_back("chassis status maps to " + robot_teleop::ToString(chassis_state) +
                          ": " + chassis_state_.status);
      }
    }

    auto resolved = robot_teleop::ResolveHighestPriority(states);
    if (!sensor_stale_stop_ && resolved == robot_teleop::SafetyState::kSensorStale) {
      resolved = robot_teleop::SafetyState::kNormal;
    }
    if (!localization_lost_stop_ && resolved == robot_teleop::SafetyState::kLocalizationLost) {
      resolved = robot_teleop::SafetyState::kNormal;
    }
    if (!chassis_fault_stop_ && resolved == robot_teleop::SafetyState::kChassisFault) {
      resolved = robot_teleop::SafetyState::kNormal;
    }
    if (!communication_lost_stop_ &&
        resolved == robot_teleop::SafetyState::kCommunicationLost) {
      resolved = robot_teleop::SafetyState::kNormal;
    }

    if (reason != nullptr) {
      *reason = JoinReasons(reasons);
    }
    return resolved;
  }

  void PublishUnifiedSafetyState(const robot_teleop::SafetyState state,
                                 const std::string& reason) {
    std_msgs::msg::String state_msg;
    state_msg.data = robot_teleop::ToString(state);
    safety_state_pub_->publish(state_msg);

    std_msgs::msg::String reason_msg;
    reason_msg.data = reason;
    safety_reason_pub_->publish(reason_msg);
  }

  void PublishState(const robot_teleop::SafetyState safety_state,
                    const std::string& safety_reason) {
    std_msgs::msg::String active_msg;
    const std::string state_hint =
        active_source_hint_ == "twist_mux" ? "twist_mux" : "twist_mux:" + active_source_hint_;
    active_msg.data = robot_teleop::DescribeCommandState(
        safety_state == robot_teleop::SafetyState::kEmergencyStop,
        safety_state == robot_teleop::SafetyState::kChassisFault ||
            safety_state == robot_teleop::SafetyState::kCommunicationLost ||
            safety_state == robot_teleop::SafetyState::kLocalizationLost ||
            safety_state == robot_teleop::SafetyState::kSensorStale ||
            safety_state == robot_teleop::SafetyState::kRecovery,
        manual_takeover_, state_hint);
    active_pub_->publish(active_msg);

    std_msgs::msg::Bool estop_msg;
    estop_msg.data = safety_state == robot_teleop::SafetyState::kEmergencyStop;
    estop_state_pub_->publish(estop_msg);

    std_msgs::msg::Bool takeover_msg;
    takeover_msg.data = manual_takeover_;
    takeover_state_pub_->publish(takeover_msg);
    takeover_lock_pub_->publish(takeover_msg);

    PublishUnifiedSafetyState(safety_state, safety_reason);
  }

  void Tick() {
    std::string safety_reason;
    const auto safety_state = ResolveSafetyState(&safety_reason);
    PublishState(safety_state, safety_reason);

    robot_teleop::CmdVelSafetyGateRequest gate_request;
    gate_request.input_twist = received_ ? last_twist_ : robot_teleop::MakeZeroTwist();
    gate_request.emergency_stop = false;
    gate_request.runtime_safety_stop = false;
    gate_request.input_available = received_;
    gate_request.input_fresh = !WatchdogExpired();
    gate_request.dynamic_speed_limit_mps = 0.0;
    gate_request.dynamic_angular_limit_radps = 0.0;
    const auto gate_decision = robot_teleop::EvaluateCmdVelSafetyGate(gate_request);

    if (gate_decision.stop_reason == robot_teleop::CmdVelStopReason::kInputUnavailable) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "no muxed cmd_vel received on '%s'; publishing zero twist",
                           input_topic_.c_str());
    } else if (gate_decision.stop_reason == robot_teleop::CmdVelStopReason::kInputStale) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "muxed cmd_vel on '%s' watchdog expired; publishing zero twist",
                           input_topic_.c_str());
    }
    const double speed_limit_mps =
        dynamic_speed_limit_mps_ > 0.0 ? std::min(dynamic_speed_limit_mps_,
                                                  speed_limited_max_linear_mps_)
                                       : speed_limited_max_linear_mps_;
    auto output_twist = robot_teleop::ApplySafetyPolicy(
        gate_decision.output_twist, safety_state, speed_limit_mps,
        speed_limited_max_angular_radps_, emergency_stop_requires_reset_);
    if (safety_state == robot_teleop::SafetyState::kNormal && dynamic_speed_limit_mps_ > 0.0) {
      output_twist = robot_teleop::ApplyDynamicSpeedLimit(
          output_twist, dynamic_speed_limit_mps_, dynamic_angular_limit_radps_);
    }
    if (robot_teleop::PolicyForState(safety_state, emergency_stop_requires_reset_)
            .force_zero_cmd_vel) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "safety state %s forces zero cmd_vel: %s",
                           robot_teleop::ToString(safety_state).c_str(),
                           safety_reason.c_str());
    }
    cmd_pub_->publish(output_twist);
  }

  std::string input_topic_;
  std::string active_source_hint_;
  std::string localization_health_topic_;
  std::string scan_topic_;
  std::string odom_topic_;
  std::string chassis_state_topic_;
  std::string safety_state_topic_;
  std::string safety_reason_topic_;
  int watchdog_timeout_ms_;
  double scan_timeout_sec_;
  double odom_timeout_sec_;
  double localization_timeout_sec_;
  double chassis_state_timeout_sec_;
  double safety_startup_grace_sec_;
  bool emergency_stop_ = false;
  bool emergency_stop_latched_ = false;
  bool manual_takeover_ = false;
  bool runtime_safety_stop_ = false;
  bool received_ = false;
  bool localization_seen_ = false;
  bool scan_seen_ = false;
  bool odom_seen_ = false;
  bool chassis_seen_ = false;
  bool localization_lost_stop_ = true;
  bool sensor_stale_stop_ = true;
  bool chassis_fault_stop_ = true;
  bool communication_lost_stop_ = true;
  bool emergency_stop_requires_reset_ = true;
  double dynamic_speed_limit_mps_ = 0.0;
  double dynamic_angular_limit_radps_ = 0.6;
  double speed_limited_max_linear_mps_ = 0.15;
  double speed_limited_max_angular_radps_ = 0.4;
  std::string localization_health_;
  rclcpp::Time last_stamp_;
  rclcpp::Time started_stamp_;
  rclcpp::Time localization_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time scan_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time chassis_stamp_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::Twist last_twist_;
  robot_interfaces::msg::ChassisState chassis_state_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr takeover_sub_;
  rclcpp::Subscription<robot_interfaces::msg::SafetyState>::SharedPtr safety_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr localization_health_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<robot_interfaces::msg::ChassisState>::SharedPtr chassis_state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr safety_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr safety_reason_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr takeover_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr takeover_lock_pub_;
  rclcpp::Service<robot_interfaces_core::srv::SetControlMode>::SharedPtr v2_set_source_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_estop_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr clear_estop_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelSafetyGateNode>());
  rclcpp::shutdown();
  return 0;
}
