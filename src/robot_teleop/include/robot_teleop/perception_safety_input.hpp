#pragma once

#include <cmath>
#include <cstdint>
#include <string>

#include "robot_teleop/cmd_vel_safety.hpp"

namespace robot_teleop {

enum class PerceptionSafetyInputState : std::uint8_t {
  kClear = 0,
  kSpeedLimited = 1,
  kStop = 2,
};

struct PerceptionSafetySnapshot {
  bool received = false;
  PerceptionSafetyInputState state = PerceptionSafetyInputState::kClear;
  std::int64_t event_stamp_ns = 0;
  std::int64_t last_clock_ns = 0;
  std::uint32_t target_id = 0U;
  double distance_m = -1.0;
  std::string event_type = "CLEAR";
  std::string reason = "PERCEPTION_CLEAR";
};

inline bool ValidatePerceptionSafetyPayload(
    const std::uint8_t state, const std::uint32_t target_id,
    const std::string& class_name, const std::string& event_type,
    const double distance_m, const std::string& source) {
  if (source != "robot_perception" || state > 2U) {
    return false;
  }
  if (state == 0U) {
    return event_type == "CLEAR";
  }
  if (target_id == 0U || class_name != "person" ||
      !std::isfinite(distance_m) || distance_m < 0.0) {
    return false;
  }
  if (state == 1U) {
    return event_type == "PERSON_NEAR";
  }
  return event_type == "PERSON_TOO_CLOSE" ||
         event_type == "PERSON_IN_DANGER_ZONE";
}

inline bool UpdatePerceptionSafetySnapshot(
    PerceptionSafetySnapshot* snapshot, const std::uint8_t state,
    const std::uint32_t target_id, const std::string& class_name,
    const std::string& event_type, const double distance_m,
    const std::string& source, const std::string& reason,
    const std::int64_t event_stamp_ns, const std::int64_t now_ns,
    const double timeout_sec) {
  if (snapshot == nullptr || !std::isfinite(timeout_sec) || timeout_sec <= 0.0) {
    return false;
  }
  if (now_ns < snapshot->last_clock_ns) {
    *snapshot = PerceptionSafetySnapshot{};
  }
  snapshot->last_clock_ns = now_ns;
  const auto timeout_ns = static_cast<std::int64_t>(timeout_sec * 1.0e9);
  if (!ValidatePerceptionSafetyPayload(
          state, target_id, class_name, event_type, distance_m, source) ||
      event_stamp_ns <= 0 || event_stamp_ns > now_ns ||
      now_ns - event_stamp_ns > timeout_ns ||
      (snapshot->received && event_stamp_ns < snapshot->event_stamp_ns)) {
    return false;
  }
  snapshot->received = true;
  snapshot->state = static_cast<PerceptionSafetyInputState>(state);
  snapshot->event_stamp_ns = event_stamp_ns;
  snapshot->target_id = target_id;
  snapshot->distance_m = distance_m;
  snapshot->event_type = event_type;
  snapshot->reason = reason.empty() ? "PERCEPTION_" + event_type : reason;
  return true;
}

inline bool PerceptionSafetyInputIsFresh(
    const PerceptionSafetySnapshot& snapshot, const std::int64_t now_ns,
    const double timeout_sec) {
  if (!snapshot.received || !std::isfinite(timeout_sec) || timeout_sec <= 0.0 ||
      now_ns < snapshot.event_stamp_ns) {
    return false;
  }
  return static_cast<double>(now_ns - snapshot.event_stamp_ns) * 1.0e-9 <=
         timeout_sec;
}

inline SafetyState PerceptionSafetyContribution(
    const PerceptionSafetySnapshot& snapshot, const std::int64_t now_ns,
    const double timeout_sec) {
  if (!PerceptionSafetyInputIsFresh(snapshot, now_ns, timeout_sec)) {
    return SafetyState::kNormal;
  }
  switch (snapshot.state) {
    case PerceptionSafetyInputState::kClear:
      return SafetyState::kNormal;
    case PerceptionSafetyInputState::kSpeedLimited:
      return SafetyState::kSpeedLimited;
    case PerceptionSafetyInputState::kStop:
      return SafetyState::kPerceptionStop;
  }
  return SafetyState::kNormal;
}

}  // namespace robot_teleop
