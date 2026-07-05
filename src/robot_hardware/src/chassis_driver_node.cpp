#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_hardware/chassis_backend_factory.hpp"
#include "robot_hardware/chassis_kinematics.hpp"
#include "robot_hardware/chassis_system_adapter.hpp"
#include "robot_interfaces/msg/chassis_state.hpp"
#include "robot_interfaces_core/srv/set_chassis_mode.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

namespace {

geometry_msgs::msg::Quaternion QuaternionFromYaw(const double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

}  // namespace

class ChassisDriverNode final : public rclcpp::Node {
 public:
  ChassisDriverNode() : Node("chassis_driver_node") {
    requested_backend_ = declare_parameter<std::string>("backend", "mock");
    mode_ = declare_parameter<std::string>("mode", "velocity");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    publish_odom_ = declare_parameter<bool>("publish_odom", true);
    wheel_odom_topic_ = declare_parameter<std::string>("wheel_odom_topic", "/wheel/odom");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    backend_config_.serial_device = declare_parameter<std::string>("serial_device", "/dev/ttyUSB0");
    backend_config_.serial_baud = declare_parameter<int>("serial_baud", 115200);
    backend_config_.udp_host = declare_parameter<std::string>("udp_host", "192.168.1.10");
    backend_config_.udp_port = declare_parameter<int>("udp_port", 9000);
    backend_config_.protocol = declare_parameter<std::string>("protocol", "text");
    kinematics_model_ =
        robot_hardware::NormalizeKinematicsModel(declare_parameter<std::string>("chassis_type", "diff_drive"));
    wheel_diameter_m_ = declare_parameter<double>("wheel_diameter_m", 0.15);
    wheel_base_m_ = declare_parameter<double>("wheel_base_m", 0.42);
    track_width_m_ = declare_parameter<double>("track_width_m", 0.36);
    io_timeout_ms_ = declare_parameter<int>("io_timeout_ms", 500);
    heartbeat_period_ms_ = declare_parameter<int>("heartbeat_period_ms", 500);
    heartbeat_timeout_ms_ = declare_parameter<int>("heartbeat_timeout_ms", 1500);
    command_timeout_ms_ = declare_parameter<int>("command_timeout_ms", 500);
    command_max_linear_velocity_mps_ =
        declare_parameter<double>("command_max_linear_velocity_mps", 0.0);
    command_max_angular_velocity_radps_ =
        declare_parameter<double>("command_max_angular_velocity_radps", 0.0);

    robot_hardware::ChassisSystemAdapterConfig adapter_config;
    adapter_config.kinematics_model = kinematics_model_;
    adapter_config.protocol = backend_config_.protocol;
    adapter_config.wheel_diameter_m = wheel_diameter_m_;
    adapter_config.wheel_base_m = wheel_base_m_;
    adapter_config.track_width_m = track_width_m_;
    adapter_config.fallback_battery_voltage = battery_voltage_;
    auto backend = robot_hardware::CreateChassisBackend(requested_backend_, backend_config_);
    backend_name_ = backend->Name();
    chassis_adapter_ =
        std::make_unique<robot_hardware::ChassisSystemAdapter>(std::move(backend), adapter_config);
    system_state_ = chassis_adapter_->State();
    if (requested_backend_ != backend_name_) {
      RCLCPP_WARN(
          get_logger(), "unknown chassis backend '%s', falling back to mock",
          requested_backend_.c_str());
    }
    imu_source_ = declare_parameter<std::string>("imu_source", "auto");
    publish_synthetic_imu_ = ResolvePublishSyntheticImu();

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, [this](geometry_msgs::msg::Twist::SharedPtr msg) { HandleCommand(*msg); });
    wheel_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(wheel_odom_topic_, 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    if (publish_synthetic_imu_) {
      imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
    }
    state_pub_ = create_publisher<robot_interfaces::msg::ChassisState>("/chassis/state", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    clear_odom_srv_ = create_service<std_srvs::srv::Empty>(
        "/clear_odometry",
        [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
               std::shared_ptr<std_srvs::srv::Empty::Response>) { ResetOdometry(); });
    v2_set_mode_srv_ = create_service<robot_interfaces_core::srv::SetChassisMode>(
        "/v2/set_chassis_mode",
        [this](const std::shared_ptr<robot_interfaces_core::srv::SetChassisMode::Request> request,
               std::shared_ptr<robot_interfaces_core::srv::SetChassisMode::Response> response) {
          SetChassisMode(request, response);
        });

    last_update_ = now();
    last_packet_stamp_ = now();
    last_heartbeat_stamp_ = now();
    OpenBackend();
    timer_ = create_wall_timer(20ms, [this]() { Tick(); });
  }

 private:
  void OpenBackend() {
    std::string error;
    backend_open_ = chassis_adapter_->Open(&error);
    system_state_ = chassis_adapter_->State();
    if (!backend_open_) {
      RCLCPP_ERROR(get_logger(), "%s", error.c_str());
      return;
    }
    if (backend_name_ == "serial") {
      RCLCPP_INFO(
          get_logger(), "serial chassis backend opened: device=%s baud=%d",
          backend_config_.serial_device.c_str(), backend_config_.serial_baud);
    } else if (backend_name_ == "udp") {
      RCLCPP_INFO(
          get_logger(), "UDP chassis backend opened: host=%s port=%d",
          backend_config_.udp_host.c_str(), backend_config_.udp_port);
    } else {
      connected_ = system_state_.connected;
      RCLCPP_INFO(get_logger(), "mock chassis backend opened");
    }
  }

  void HandleCommand(const geometry_msgs::msg::Twist& twist) {
    cmd_.linear_x_mps = twist.linear.x;
    cmd_.linear_y_mps = twist.linear.y;
    cmd_.angular_z_radps = twist.angular.z;
    last_command_stamp_ = now();
    received_command_ = true;
    command_timeout_active_ = false;
    if (!backend_open_) {
      local_fault_code_ = robot_hardware::ChassisFaultCode::kBackendDisconnected;
      return;
    }
    std::string error;
    const auto frame = BuildCommandFrame(last_command_stamp_, cmd_, false);
    if (!chassis_adapter_->Write(frame, &error) && !error.empty()) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "%s command write failed: %s",
          backend_name_.c_str(), error.c_str());
      local_fault_code_ = robot_hardware::ChassisFaultCode::kBackendDisconnected;
      return;
    }
    local_fault_code_ = robot_hardware::ChassisFaultCode::kNone;
  }

  template <typename RequestT, typename ResponseT>
  void SetChassisMode(
      const std::shared_ptr<RequestT>& request, const std::shared_ptr<ResponseT>& response) {
    const std::string normalized = robot_hardware::NormalizeKinematicsModel(request->mode);
    if (request->mode == "diff_drive" || request->mode == "mecanum" ||
        request->mode == "omni") {
      mode_ = normalized;
      kinematics_model_ = normalized;
      chassis_adapter_->SetKinematicsModel(kinematics_model_);
      response->success = true;
      response->message = "chassis mode set to " + mode_ + ", kinematics=" + kinematics_model_;
      return;
    }
    if (request->mode == "ackermann" || request->mode == "acker" ||
        request->mode == "four_ws4wd" || request->mode == "4ws4wd" ||
        request->mode == "four_wheel_steer") {
      mode_ = normalized;
      kinematics_model_ = normalized;
      chassis_adapter_->SetKinematicsModel(kinematics_model_);
      response->success = true;
      response->message =
          "requested chassis mode " + request->mode +
          " is not implemented; using diff_drive kinematics";
      return;
    }
    response->success = false;
    response->message =
        "unsupported chassis mode: " + request->mode +
        " (supported: diff_drive, mecanum; ackermann/four_ws4wd degrade to diff_drive)";
  }

  void Tick() {
    const auto stamp = now();
    const double dt = std::max(0.001, (stamp - last_update_).seconds());
    last_update_ = stamp;

    EnforceCommandTimeout(stamp);
    SendHeartbeatIfDue(stamp);
    ReadChassisSystem(dt, stamp);
    IntegrateWheelOdometry(dt);

    PublishOdometry(stamp);
    if (publish_synthetic_imu_) {
      PublishImu(stamp);
    }
    PublishState(stamp);
  }

  void ReadChassisSystem(const double dt, const rclcpp::Time& stamp) {
    const auto previous_packet_sequence = system_state_.packet_sequence;
    std::string error;
    const bool read_ok = chassis_adapter_->Read(dt, &system_state_, &error);
    backend_open_ = system_state_.backend_open;
    if (!error.empty()) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "%s backend read failed: %s",
          backend_name_.c_str(), error.c_str());
      if (error.find("malformed") != std::string::npos) {
        local_fault_code_ = robot_hardware::ChassisFaultCode::kMalformedPacket;
      }
    }
    if (system_state_.packet_sequence != previous_packet_sequence) {
      last_packet_stamp_ = stamp;
    }
    if (backend_name_ == "mock") {
      connected_ = read_ok;
    } else {
      const int active_timeout_ms = heartbeat_timeout_ms_ > 0 ? heartbeat_timeout_ms_
                                                              : io_timeout_ms_;
      connected_ = backend_open_ &&
                   ((stamp - last_packet_stamp_).nanoseconds() / 1000000 <= active_timeout_ms);
    }

    if (!backend_open_) {
      local_fault_code_ = robot_hardware::ChassisFaultCode::kBackendDisconnected;
    } else if (!connected_) {
      local_fault_code_ = robot_hardware::ChassisFaultCode::kHeartbeatTimeout;
    } else if (system_state_.emergency_stop) {
      local_fault_code_ = robot_hardware::ChassisFaultCode::kEmergencyStopActive;
    } else if (system_state_.fault_code != robot_hardware::ChassisFaultCode::kNone) {
      local_fault_code_ = system_state_.fault_code;
    } else if (!command_timeout_active_) {
      local_fault_code_ = robot_hardware::ChassisFaultCode::kNone;
    }
  }

  robot_hardware::ChassisCommandFrameV2 BuildCommandFrame(
      const rclcpp::Time& stamp, const robot_hardware::ChassisCommand& command,
      const bool emergency_stop) {
    robot_hardware::ChassisCommandFrameV2 frame;
    frame.seq = ++tx_sequence_;
    frame.timestamp_sec = stamp.seconds();
    frame.mode = mode_;
    frame.command = command;
    frame.max_linear_velocity_mps = command_max_linear_velocity_mps_;
    frame.max_angular_velocity_radps = command_max_angular_velocity_radps_;
    frame.emergency_stop = emergency_stop;
    return frame;
  }

  void EnforceCommandTimeout(const rclcpp::Time& stamp) {
    if (!received_command_ || command_timeout_ms_ <= 0 || command_timeout_active_) {
      return;
    }
    if ((stamp - last_command_stamp_).nanoseconds() / 1000000 <= command_timeout_ms_) {
      return;
    }

    robot_hardware::ChassisCommand stop_command;
    cmd_ = stop_command;
    command_timeout_active_ = true;
    local_fault_code_ = robot_hardware::ChassisFaultCode::kCommandTimeout;
    if (!backend_open_) {
      return;
    }

    std::string error;
    const auto frame = BuildCommandFrame(stamp, stop_command, false);
    if (!chassis_adapter_->Write(frame, &error) && !error.empty()) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "%s timeout stop write failed: %s",
          backend_name_.c_str(), error.c_str());
      local_fault_code_ = robot_hardware::ChassisFaultCode::kBackendDisconnected;
      return;
    }
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "cmd_vel timeout after %d ms; sent zero velocity command", command_timeout_ms_);
  }

  void SendHeartbeatIfDue(const rclcpp::Time& stamp) {
    if (heartbeat_period_ms_ <= 0 || !backend_open_) {
      return;
    }
    if ((stamp - last_heartbeat_stamp_).nanoseconds() / 1000000 < heartbeat_period_ms_) {
      return;
    }
    last_heartbeat_stamp_ = stamp;

    robot_hardware::ChassisHeartbeatPacket heartbeat;
    heartbeat.seq = ++tx_sequence_;
    heartbeat.timestamp_sec = stamp.seconds();
    heartbeat.source = "chassis_driver_node";
    std::string error;
    if (!chassis_adapter_->WriteHeartbeat(heartbeat, &error) && !error.empty()) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "%s heartbeat write failed: %s",
          backend_name_.c_str(), error.c_str());
      local_fault_code_ = robot_hardware::ChassisFaultCode::kBackendDisconnected;
    }
  }

  bool ResolvePublishSyntheticImu() {
    if (imu_source_ == "auto") {
      return backend_name_ == "mock";
    }
    if (imu_source_ == "synthetic" || imu_source_ == "fake") {
      return true;
    }
    if (imu_source_ == "none" || imu_source_ == "off" || imu_source_ == "disabled") {
      return false;
    }
    RCLCPP_WARN(
        get_logger(), "unknown imu_source '%s'; supported values: auto, synthetic, none",
        imu_source_.c_str());
    imu_source_ = "none";
    return false;
  }

  void IntegrateWheelOdometry(const double dt) {
    wheel_odom_command_ = robot_hardware::EstimateCommandFromWheelSpeeds(
        system_state_.wheel_speeds, kinematics_model_, wheel_diameter_m_, wheel_base_m_,
        track_width_m_);
    robot_hardware::PlanarPose pose{wheel_x_, wheel_y_, wheel_yaw_};
    pose = robot_hardware::IntegratePlanarMotion(pose, wheel_odom_command_, dt);
    wheel_x_ = pose.x_m;
    wheel_y_ = pose.y_m;
    wheel_yaw_ = pose.yaw_rad;
  }

  void PublishOdometry(const rclcpp::Time& stamp) {
    nav_msgs::msg::Odometry wheel_odom;
    wheel_odom.header.stamp = stamp;
    wheel_odom.header.frame_id = "odom";
    wheel_odom.child_frame_id = "base_footprint";
    wheel_odom.pose.pose.position.x = wheel_x_;
    wheel_odom.pose.pose.position.y = wheel_y_;
    wheel_odom.pose.pose.orientation = QuaternionFromYaw(wheel_yaw_);
    wheel_odom.twist.twist.linear.x = wheel_odom_command_.linear_x_mps;
    wheel_odom.twist.twist.linear.y = wheel_odom_command_.linear_y_mps;
    wheel_odom.twist.twist.angular.z = wheel_odom_command_.angular_z_radps;
    wheel_odom_pub_->publish(wheel_odom);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    odom.pose.pose.position.x = system_state_.x_m;
    odom.pose.pose.position.y = system_state_.y_m;
    odom.pose.pose.orientation = QuaternionFromYaw(system_state_.yaw_rad);
    odom.twist.twist.linear.x = system_state_.linear_x_mps;
    odom.twist.twist.linear.y = system_state_.linear_y_mps;
    odom.twist.twist.angular.z = system_state_.angular_z_radps;
    if (publish_odom_) {
      odom_pub_->publish(odom);
    }

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header = odom.header;
      tf.child_frame_id = odom.child_frame_id;
      tf.transform.translation.x = system_state_.x_m;
      tf.transform.translation.y = system_state_.y_m;
      tf.transform.rotation = odom.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf);
    }
  }

  void PublishImu(const rclcpp::Time& stamp) {
    if (!imu_pub_) {
      return;
    }
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = "imu_link";
    imu.orientation = QuaternionFromYaw(system_state_.yaw_rad);
    imu.angular_velocity.z = system_state_.angular_z_radps;
    double linear_acceleration_x = 0.0;
    if (has_previous_imu_sample_) {
      const double dt = (stamp - previous_imu_stamp_).seconds();
      if (dt > 1e-6) {
        linear_acceleration_x = (system_state_.linear_x_mps - previous_imu_linear_velocity_) / dt;
      }
    }
    imu.linear_acceleration.x = linear_acceleration_x;
    previous_imu_linear_velocity_ = system_state_.linear_x_mps;
    previous_imu_stamp_ = stamp;
    has_previous_imu_sample_ = true;
    imu_pub_->publish(imu);
  }

  void PublishState(const rclcpp::Time& stamp) {
    robot_interfaces::msg::ChassisState state;
    state.header.stamp = stamp;
    state.backend = backend_name_;
    state.kinematics_model = kinematics_model_;
    state.connected = connected_;
    state.battery_voltage = system_state_.battery_voltage;
    state.linear_velocity = system_state_.linear_x_mps;
    state.angular_velocity = system_state_.angular_z_radps;
    const auto reported_fault = local_fault_code_ != robot_hardware::ChassisFaultCode::kNone
                                    ? local_fault_code_
                                    : system_state_.fault_code;
    std::ostringstream status;
    if (backend_open_) {
      status << system_state_.hardware_status << ":" << mode_;
    } else {
      status << "backend_unavailable:" << mode_;
    }
    status << ":fault_code=" << robot_hardware::FaultCodeName(reported_fault) << "("
           << robot_hardware::FaultCodeValue(reported_fault) << ")"
           << ":rx_seq=" << system_state_.last_rx_seq
           << ":rx_ts=" << system_state_.last_rx_timestamp_sec
           << ":estop=" << (system_state_.emergency_stop ? 1 : 0)
           << ":temp_c=" << system_state_.temperature_c
           << ":wheels_rpm=" << system_state_.wheel_speeds.rpm[0] << ","
           << system_state_.wheel_speeds.rpm[1] << ","
           << system_state_.wheel_speeds.rpm[2] << ","
           << system_state_.wheel_speeds.rpm[3];
    state.status = status.str();
    state_pub_->publish(state);
  }

  void ResetOdometry() {
    chassis_adapter_->ResetOdometry();
    system_state_ = chassis_adapter_->State();
    wheel_x_ = 0.0;
    wheel_y_ = 0.0;
    wheel_yaw_ = 0.0;
  }

  std::string requested_backend_;
  std::string backend_name_;
  robot_hardware::ChassisBackendConfig backend_config_;
  std::unique_ptr<robot_hardware::ChassisSystemAdapter> chassis_adapter_;
  robot_hardware::ChassisSystemState system_state_;
  std::string mode_;
  std::string kinematics_model_;
  std::string wheel_odom_topic_;
  std::string odom_topic_;
  std::string imu_source_;
  int io_timeout_ms_;
  int heartbeat_period_ms_;
  int heartbeat_timeout_ms_;
  int command_timeout_ms_;
  bool publish_tf_;
  bool publish_odom_;
  bool publish_synthetic_imu_ = false;
  bool backend_open_ = false;
  bool connected_ = false;
  double wheel_x_ = 0.0;
  double wheel_y_ = 0.0;
  double wheel_yaw_ = 0.0;
  double battery_voltage_ = 24.0;
  double wheel_diameter_m_ = 0.15;
  double wheel_base_m_ = 0.42;
  double track_width_m_ = 0.36;
  double command_max_linear_velocity_mps_ = 0.0;
  double command_max_angular_velocity_radps_ = 0.0;
  uint64_t tx_sequence_ = 0;
  robot_hardware::ChassisCommand cmd_;
  robot_hardware::ChassisCommand wheel_odom_command_;
  robot_hardware::ChassisFaultCode local_fault_code_ = robot_hardware::ChassisFaultCode::kNone;
  rclcpp::Time last_update_;
  rclcpp::Time last_packet_stamp_;
  rclcpp::Time last_command_stamp_;
  rclcpp::Time last_heartbeat_stamp_;
  rclcpp::Time previous_imu_stamp_;
  double previous_imu_linear_velocity_ = 0.0;
  bool has_previous_imu_sample_ = false;
  bool received_command_ = false;
  bool command_timeout_active_ = false;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<robot_interfaces::msg::ChassisState>::SharedPtr state_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clear_odom_srv_;
  rclcpp::Service<robot_interfaces_core::srv::SetChassisMode>::SharedPtr v2_set_mode_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChassisDriverNode>());
  rclcpp::shutdown();
  return 0;
}
