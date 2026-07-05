#pragma once

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"

namespace robot_teleop {

enum class SafetyState {
  kNormal,
  kManualTakeover,
  kSpeedLimited,
  kSensorStale,
  kLocalizationLost,
  kChassisFault,
  kCommunicationLost,
  kEmergencyStop,
  kRecovery,
};

struct SafetyPolicy {
  bool allow_cmd_vel = true;
  bool force_zero_cmd_vel = false;
  bool speed_limited = false;
  bool requires_manual_reset = false;
};

inline std::string ToString(const SafetyState state) {
  switch (state) {
    case SafetyState::kNormal:
      return "NORMAL";
    case SafetyState::kManualTakeover:
      return "MANUAL_TAKEOVER";
    case SafetyState::kSpeedLimited:
      return "SPEED_LIMITED";
    case SafetyState::kSensorStale:
      return "SENSOR_STALE";
    case SafetyState::kLocalizationLost:
      return "LOCALIZATION_LOST";
    case SafetyState::kChassisFault:
      return "CHASSIS_FAULT";
    case SafetyState::kCommunicationLost:
      return "COMMUNICATION_LOST";
    case SafetyState::kEmergencyStop:
      return "EMERGENCY_STOP";
    case SafetyState::kRecovery:
      return "RECOVERY";
  }
  return "NORMAL";
}

inline int SafetyPriority(const SafetyState state) {
  switch (state) {
    case SafetyState::kEmergencyStop:
      return 90;
    case SafetyState::kCommunicationLost:
      return 80;
    case SafetyState::kChassisFault:
      return 70;
    case SafetyState::kLocalizationLost:
      return 60;
    case SafetyState::kSensorStale:
      return 50;
    case SafetyState::kManualTakeover:
      return 40;
    case SafetyState::kSpeedLimited:
      return 30;
    case SafetyState::kRecovery:
      return 20;
    case SafetyState::kNormal:
      return 10;
  }
  return 10;
}

inline SafetyState ResolveHighestPriority(const std::vector<SafetyState>& states) {
  SafetyState resolved = SafetyState::kNormal;
  for (const auto state : states) {
    if (SafetyPriority(state) > SafetyPriority(resolved)) {
      resolved = state;
    }
  }
  return resolved;
}

inline SafetyPolicy PolicyForState(const SafetyState state,
                                   const bool emergency_stop_requires_reset = true) {
  SafetyPolicy policy;
  switch (state) {
    case SafetyState::kEmergencyStop:
      policy.allow_cmd_vel = false;
      policy.force_zero_cmd_vel = true;
      policy.requires_manual_reset = emergency_stop_requires_reset;
      return policy;
    case SafetyState::kCommunicationLost:
    case SafetyState::kChassisFault:
    case SafetyState::kLocalizationLost:
    case SafetyState::kSensorStale:
    case SafetyState::kRecovery:
      policy.allow_cmd_vel = false;
      policy.force_zero_cmd_vel = true;
      return policy;
    case SafetyState::kSpeedLimited:
      policy.speed_limited = true;
      return policy;
    case SafetyState::kManualTakeover:
    case SafetyState::kNormal:
      return policy;
  }
  return policy;
}

inline std::string ExtractFaultCodeName(const std::string& status) {
  const std::string marker = "fault_code=";
  const auto begin = status.find(marker);
  if (begin == std::string::npos) {
    return "";
  }
  const auto name_begin = begin + marker.size();
  const auto paren = status.find('(', name_begin);
  const auto colon = status.find(':', name_begin);
  std::size_t name_end = std::string::npos;
  if (paren != std::string::npos && colon != std::string::npos) {
    name_end = std::min(paren, colon);
  } else if (paren != std::string::npos) {
    name_end = paren;
  } else if (colon != std::string::npos) {
    name_end = colon;
  }
  return status.substr(name_begin, name_end == std::string::npos ? name_end : name_end - name_begin);
}

inline bool StatusReportsEstop(const std::string& status) {
  return status.find("estop=1") != std::string::npos ||
         status.find("ESTOP_ACTIVE") != std::string::npos;
}

inline SafetyState SafetyStateFromChassisStatus(const bool connected,
                                                const std::string& status) {
  if (!connected) {
    return SafetyState::kCommunicationLost;
  }
  if (StatusReportsEstop(status)) {
    return SafetyState::kEmergencyStop;
  }
  const auto fault_code = ExtractFaultCodeName(status);
  if (fault_code.empty() || fault_code == "NONE") {
    return SafetyState::kNormal;
  }
  if (fault_code == "CMD_TIMEOUT") {
    return SafetyState::kSensorStale;
  }
  if (fault_code == "HEARTBEAT_TIMEOUT" || fault_code == "BACKEND_DISCONNECTED") {
    return SafetyState::kCommunicationLost;
  }
  if (fault_code == "MALFORMED_PACKET") {
    return SafetyState::kChassisFault;
  }
  if (fault_code == "ESTOP_ACTIVE") {
    return SafetyState::kEmergencyStop;
  }
  return SafetyState::kChassisFault;
}

inline SafetyState SafetyStateFromLocalizationHealth(const std::string& health) {
  if (health.find("LOCALIZATION_LOST") != std::string::npos) {
    return SafetyState::kLocalizationLost;
  }
  if (health.find("LOCALIZATION_RECOVERING") != std::string::npos ||
      health.find("LOCALIZATION_RECOVERED") != std::string::npos) {
    return SafetyState::kRecovery;
  }
  if (health.find("LOCALIZATION_UNSTABLE") != std::string::npos) {
    return SafetyState::kSpeedLimited;
  }
  return SafetyState::kNormal;
}

inline geometry_msgs::msg::Twist MakeZeroTwist() {
  geometry_msgs::msg::Twist twist;
  return twist;
}

inline geometry_msgs::msg::Twist ApplyDynamicSpeedLimit(
    geometry_msgs::msg::Twist twist, const double linear_limit_mps,
    const double angular_limit_radps) {
  if (linear_limit_mps <= 0.0) {
    return twist;
  }

  const double linear_speed = std::hypot(twist.linear.x, twist.linear.y);
  if (linear_speed > linear_limit_mps && linear_speed > 0.0) {
    const double scale = linear_limit_mps / linear_speed;
    twist.linear.x *= scale;
    twist.linear.y *= scale;
    twist.angular.z *= scale;
  }

  if (angular_limit_radps > 0.0) {
    twist.angular.z = std::clamp(twist.angular.z, -angular_limit_radps, angular_limit_radps);
  }
  return twist;
}

inline geometry_msgs::msg::Twist ApplySafetyPolicy(
    const geometry_msgs::msg::Twist& twist, const SafetyState state,
    const double speed_limited_max_linear_mps, const double speed_limited_max_angular_radps,
    const bool emergency_stop_requires_reset = true) {
  const auto policy = PolicyForState(state, emergency_stop_requires_reset);
  if (policy.force_zero_cmd_vel) {
    return MakeZeroTwist();
  }
  if (policy.speed_limited) {
    return ApplyDynamicSpeedLimit(
        twist, speed_limited_max_linear_mps, speed_limited_max_angular_radps);
  }
  return twist;
}

enum class CmdVelStopReason {
  kNone,
  kEmergencyStop,
  kRuntimeSafetyStop,
  kInputUnavailable,
  kInputStale,
};

struct CmdVelSafetyGateRequest {
  geometry_msgs::msg::Twist input_twist;
  bool emergency_stop = false;
  bool runtime_safety_stop = false;
  bool input_available = true;
  bool input_fresh = true;
  double dynamic_speed_limit_mps = 0.0;
  double dynamic_angular_limit_radps = 0.6;
};

struct CmdVelSafetyGateDecision {
  geometry_msgs::msg::Twist output_twist;
  CmdVelStopReason stop_reason = CmdVelStopReason::kNone;

  bool stopped() const { return stop_reason != CmdVelStopReason::kNone; }
};

inline std::string SelectCommandSource(const bool manual_takeover,
                                       const std::string& active_source) {
  return manual_takeover ? "teleop" : active_source;
}

inline std::string DescribeCommandState(const bool emergency_stop,
                                        const bool runtime_safety_stop,
                                        const bool manual_takeover,
                                        const std::string& active_source) {
  if (emergency_stop) {
    return "emergency_stop";
  }
  if (runtime_safety_stop) {
    return "safety_stop";
  }
  if (manual_takeover) {
    return "manual_takeover:teleop";
  }
  return active_source;
}

inline CmdVelSafetyGateDecision EvaluateCmdVelSafetyGate(
    const CmdVelSafetyGateRequest& request) {
  CmdVelSafetyGateDecision decision;
  if (request.emergency_stop) {
    decision.output_twist = MakeZeroTwist();
    decision.stop_reason = CmdVelStopReason::kEmergencyStop;
    return decision;
  }
  if (request.runtime_safety_stop) {
    decision.output_twist = MakeZeroTwist();
    decision.stop_reason = CmdVelStopReason::kRuntimeSafetyStop;
    return decision;
  }
  if (!request.input_available) {
    decision.output_twist = MakeZeroTwist();
    decision.stop_reason = CmdVelStopReason::kInputUnavailable;
    return decision;
  }
  if (!request.input_fresh) {
    decision.output_twist = MakeZeroTwist();
    decision.stop_reason = CmdVelStopReason::kInputStale;
    return decision;
  }

  decision.output_twist =
      ApplyDynamicSpeedLimit(request.input_twist, request.dynamic_speed_limit_mps,
                             request.dynamic_angular_limit_radps);
  decision.stop_reason = CmdVelStopReason::kNone;
  return decision;
}

}  // namespace robot_teleop
