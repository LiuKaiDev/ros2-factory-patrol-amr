#include "robot_hardware/chassis_hardware_interface.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "robot_hardware/chassis_backend_factory.hpp"

namespace robot_hardware {
namespace {

constexpr double kPi = 3.14159265358979323846;

double ParameterAsDouble(
    const hardware_interface::HardwareInfo& info, const std::string& name,
    const double fallback) {
  const auto it = info.hardware_parameters.find(name);
  if (it == info.hardware_parameters.end()) {
    return fallback;
  }
  return std::strtod(it->second.c_str(), nullptr);
}

int ParameterAsInt(
    const hardware_interface::HardwareInfo& info, const std::string& name, const int fallback) {
  const auto it = info.hardware_parameters.find(name);
  if (it == info.hardware_parameters.end()) {
    return fallback;
  }
  return static_cast<int>(std::strtol(it->second.c_str(), nullptr, 10));
}

std::string ParameterAsString(
    const hardware_interface::HardwareInfo& info, const std::string& name,
    const std::string& fallback) {
  const auto it = info.hardware_parameters.find(name);
  if (it == info.hardware_parameters.end()) {
    return fallback;
  }
  return it->second;
}

double WheelRpmFromRadPerSecond(const double radps) {
  return radps * 60.0 / (2.0 * kPi);
}

double WheelRadPerSecondFromRpm(const double rpm) { return rpm * (2.0 * kPi) / 60.0; }

}  // namespace

hardware_interface::CallbackReturn ChassisHardwareInterface::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) {
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!LoadParameters() || !ValidateJoints()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ChassisHardwareInterface::on_configure(
    const rclcpp_lifecycle::State&) {
  auto backend = CreateChassisBackend(backend_name_, backend_config_);
  adapter_ = std::make_unique<ChassisSystemAdapter>(std::move(backend), adapter_config_);
  std::string error;
  if (!adapter_->Open(&error)) {
    RCLCPP_ERROR(rclcpp::get_logger("ChassisHardwareInterface"), "%s", error.c_str());
    adapter_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ChassisHardwareInterface::on_activate(
    const rclcpp_lifecycle::State&) {
  wheel_command_radps_ = {0.0, 0.0};
  if (adapter_) {
    std::string error;
    adapter_->Write(ChassisCommand{}, &error);
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ChassisHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State&) {
  if (adapter_) {
    std::string error;
    adapter_->Write(ChassisCommand{}, &error);
    adapter_->Close();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ChassisHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.emplace_back(
      joint_names_[0], hardware_interface::HW_IF_POSITION, &wheel_position_rad_[0]);
  interfaces.emplace_back(
      joint_names_[0], hardware_interface::HW_IF_VELOCITY, &wheel_velocity_radps_[0]);
  interfaces.emplace_back(
      joint_names_[1], hardware_interface::HW_IF_POSITION, &wheel_position_rad_[1]);
  interfaces.emplace_back(
      joint_names_[1], hardware_interface::HW_IF_VELOCITY, &wheel_velocity_radps_[1]);
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
ChassisHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.emplace_back(
      joint_names_[0], hardware_interface::HW_IF_VELOCITY, &wheel_command_radps_[0]);
  interfaces.emplace_back(
      joint_names_[1], hardware_interface::HW_IF_VELOCITY, &wheel_command_radps_[1]);
  return interfaces;
}

hardware_interface::return_type ChassisHardwareInterface::read(
    const rclcpp::Time&, const rclcpp::Duration& period) {
  if (!adapter_) {
    return hardware_interface::return_type::ERROR;
  }

  ChassisSystemState state;
  std::string error;
  const double period_seconds = std::max(0.0, period.seconds());
  if (!adapter_->Read(period_seconds, &state, &error) && !error.empty()) {
    RCLCPP_WARN(
        rclcpp::get_logger("ChassisHardwareInterface"), "chassis read failed: %s",
        error.c_str());
  }
  UpdateWheelStates(state, period_seconds);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ChassisHardwareInterface::write(
    const rclcpp::Time&, const rclcpp::Duration&) {
  if (!adapter_) {
    return hardware_interface::return_type::ERROR;
  }

  std::string error;
  if (!adapter_->Write(CommandFromWheelVelocityCommands(), &error)) {
    RCLCPP_WARN(
        rclcpp::get_logger("ChassisHardwareInterface"), "chassis write failed: %s",
        error.c_str());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

bool ChassisHardwareInterface::LoadParameters() {
  backend_name_ = ParameterAsString(info_, "backend", "mock");
  backend_config_.serial_device = ParameterAsString(info_, "serial_device", "/dev/ttyUSB0");
  backend_config_.serial_baud = ParameterAsInt(info_, "serial_baud", 115200);
  backend_config_.udp_host = ParameterAsString(info_, "udp_host", "192.168.1.10");
  backend_config_.udp_port = ParameterAsInt(info_, "udp_port", 9000);
  backend_config_.protocol = ParameterAsString(info_, "protocol", "text");
  adapter_config_.protocol = backend_config_.protocol;
  adapter_config_.kinematics_model =
      ParameterAsString(info_, "chassis_type", adapter_config_.kinematics_model);
  adapter_config_.wheel_diameter_m =
      ParameterAsDouble(info_, "wheel_diameter_m", adapter_config_.wheel_diameter_m);
  adapter_config_.wheel_base_m =
      ParameterAsDouble(info_, "wheel_base_m", adapter_config_.wheel_base_m);
  adapter_config_.track_width_m =
      ParameterAsDouble(info_, "track_width_m", adapter_config_.track_width_m);
  adapter_config_.fallback_battery_voltage =
      ParameterAsDouble(info_, "fallback_battery_voltage", 24.0);
  return adapter_config_.wheel_diameter_m > 0.0 && adapter_config_.track_width_m > 0.0;
}

bool ChassisHardwareInterface::ValidateJoints() const {
  if (info_.joints.size() != joint_names_.size()) {
    RCLCPP_ERROR(
        rclcpp::get_logger("ChassisHardwareInterface"),
        "expected %zu wheel joints, got %zu", joint_names_.size(), info_.joints.size());
    return false;
  }
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    if (info_.joints[i].name != joint_names_[i]) {
      RCLCPP_ERROR(
          rclcpp::get_logger("ChassisHardwareInterface"),
          "expected joint %s at index %zu, got %s", joint_names_[i].c_str(), i,
          info_.joints[i].name.c_str());
      return false;
    }
  }
  return true;
}

ChassisCommand ChassisHardwareInterface::CommandFromWheelVelocityCommands() const {
  WheelSpeeds speeds;
  speeds.rpm[0] = WheelRpmFromRadPerSecond(wheel_command_radps_[0]);
  speeds.rpm[1] = WheelRpmFromRadPerSecond(wheel_command_radps_[1]);
  speeds.rpm[2] = speeds.rpm[0];
  speeds.rpm[3] = speeds.rpm[1];
  return EstimateCommandFromWheelSpeeds(
      speeds, adapter_config_.kinematics_model, adapter_config_.wheel_diameter_m,
      adapter_config_.wheel_base_m, adapter_config_.track_width_m);
}

void ChassisHardwareInterface::UpdateWheelStates(
    const ChassisSystemState& state, const double period_seconds) {
  wheel_velocity_radps_[0] = WheelRadPerSecondFromRpm(state.wheel_speeds.rpm[0]);
  wheel_velocity_radps_[1] = WheelRadPerSecondFromRpm(state.wheel_speeds.rpm[1]);
  wheel_position_rad_[0] += wheel_velocity_radps_[0] * period_seconds;
  wheel_position_rad_[1] += wheel_velocity_radps_[1] * period_seconds;
}

}  // namespace robot_hardware

PLUGINLIB_EXPORT_CLASS(
    robot_hardware::ChassisHardwareInterface, hardware_interface::SystemInterface)
