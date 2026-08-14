#include "robot_perception/perception_safety_policy.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace robot_perception {
namespace {

bool IsFinitePosition(const Position3D& position) {
  return std::isfinite(position.x) && std::isfinite(position.y) &&
         std::isfinite(position.z);
}

int Restrictiveness(const PerceptionSafetyLevel level) {
  return static_cast<int>(level);
}

double PlanarDistance(const Position3D& left, const Position3D& right) {
  return std::hypot(left.x - right.x, left.y - right.y);
}

bool MoreRestrictiveOrDeterministic(
    const PerceptionSafetyDecision& candidate,
    const PerceptionSafetyDecision& selected) {
  const int candidate_level = Restrictiveness(candidate.level);
  const int selected_level = Restrictiveness(selected.level);
  if (candidate_level != selected_level) {
    return candidate_level > selected_level;
  }
  if (candidate.reason != selected.reason) {
    return static_cast<int>(candidate.reason) > static_cast<int>(selected.reason);
  }
  if (candidate.distance_m != selected.distance_m) {
    return candidate.distance_m < selected.distance_m;
  }
  return candidate.target_id < selected.target_id;
}

}  // namespace

const char* PerceptionSafetyLevelName(const PerceptionSafetyLevel level) {
  switch (level) {
    case PerceptionSafetyLevel::kClear:
      return "CLEAR";
    case PerceptionSafetyLevel::kSpeedLimited:
      return "SPEED_LIMITED";
    case PerceptionSafetyLevel::kStop:
      return "STOP";
  }
  return "CLEAR";
}

const char* PerceptionSafetyReasonName(const PerceptionSafetyReason reason) {
  switch (reason) {
    case PerceptionSafetyReason::kClear:
      return "CLEAR";
    case PerceptionSafetyReason::kPersonNear:
      return "PERSON_NEAR";
    case PerceptionSafetyReason::kPersonTooClose:
      return "PERSON_TOO_CLOSE";
    case PerceptionSafetyReason::kPersonInDangerZone:
      return "PERSON_IN_DANGER_ZONE";
  }
  return "CLEAR";
}

PerceptionSafetyPolicy::PerceptionSafetyPolicy(PerceptionSafetyConfig config)
    : config_(std::move(config)) {
  if (config_.person_class.empty()) {
    throw std::invalid_argument("person_class must not be empty");
  }
  if (!std::isfinite(config_.person_stop_distance) ||
      config_.person_stop_distance <= 0.0) {
    throw std::invalid_argument(
        "person_stop_distance must be positive and finite");
  }
  if (!std::isfinite(config_.person_slow_distance) ||
      config_.person_slow_distance <= config_.person_stop_distance) {
    throw std::invalid_argument(
        "person_slow_distance must be finite and greater than person_stop_distance");
  }
  if (config_.clear_observations == 0U) {
    throw std::invalid_argument("clear_observations must be positive");
  }
  if (!std::isfinite(config_.stop_hysteresis) ||
      config_.stop_hysteresis < 0.0 ||
      !std::isfinite(config_.slow_hysteresis) ||
      config_.slow_hysteresis < 0.0) {
    throw std::invalid_argument("safety hysteresis values must be finite and nonnegative");
  }
  if (!std::isfinite(config_.max_target_age_sec) ||
      config_.max_target_age_sec <= 0.0) {
    throw std::invalid_argument("max_target_age_sec must be positive and finite");
  }
  decision_.class_name = config_.person_class;
}

PerceptionSafetyDecision PerceptionSafetyPolicy::Classify(
    const std::vector<SafetyTarget>& targets, const Position3D& robot_position,
    const std::vector<robot_navigation::MapZone>& danger_zones,
    const std::int64_t update_timestamp_ns, bool* malformed_person_input) const {
  PerceptionSafetyDecision selected;
  selected.class_name = config_.person_class;
  *malformed_person_input = false;
  if (!config_.enabled) {
    return selected;
  }
  if (update_timestamp_ns < 0 || !IsFinitePosition(robot_position)) {
    selected.input_valid = false;
    return selected;
  }

  const auto max_age_ns = static_cast<std::int64_t>(
      config_.max_target_age_sec * 1.0e9);
  bool selected_person = false;
  for (const auto& target : targets) {
    if (target.class_name != config_.person_class ||
        (target.state != TrackingState::kConfirmed &&
         target.state != TrackingState::kProcessed) ||
        target.missed_frames != 0U) {
      continue;
    }
    if (!target.depth_valid || !IsFinitePosition(target.position) ||
        target.last_observation_ns < 0 ||
        target.last_observation_ns > update_timestamp_ns) {
      *malformed_person_input = true;
      continue;
    }
    const auto age_ns = update_timestamp_ns - target.last_observation_ns;
    if (age_ns > max_age_ns) {
      continue;
    }

    PerceptionSafetyDecision candidate;
    candidate.target_id = target.target_id;
    candidate.class_name = target.class_name;
    candidate.target_position = target.position;
    candidate.distance_m = PlanarDistance(robot_position, target.position);
    if (!std::isfinite(candidate.distance_m)) {
      *malformed_person_input = true;
      continue;
    }

    for (const auto& zone : danger_zones) {
      if (zone.enabled && zone.type == "danger_zone" &&
          robot_navigation::ContainsPoint(
              zone, target.position.x, target.position.y)) {
        candidate.level = PerceptionSafetyLevel::kStop;
        candidate.reason = PerceptionSafetyReason::kPersonInDangerZone;
        candidate.zone_id = zone.zone_id;
        break;
      }
    }
    if (candidate.reason != PerceptionSafetyReason::kPersonInDangerZone) {
      double stop_threshold = config_.person_stop_distance;
      double slow_threshold = config_.person_slow_distance;
      if (decision_.level == PerceptionSafetyLevel::kStop) {
        stop_threshold += config_.stop_hysteresis;
      }
      if (decision_.level != PerceptionSafetyLevel::kClear) {
        slow_threshold += config_.slow_hysteresis;
      }
      if (candidate.distance_m < stop_threshold) {
        candidate.level = PerceptionSafetyLevel::kStop;
        candidate.reason = PerceptionSafetyReason::kPersonTooClose;
      } else if (candidate.distance_m <= slow_threshold) {
        candidate.level = PerceptionSafetyLevel::kSpeedLimited;
        candidate.reason = PerceptionSafetyReason::kPersonNear;
      }
    }

    if (!selected_person || MoreRestrictiveOrDeterministic(candidate, selected)) {
      selected = candidate;
      selected_person = true;
    }
  }
  return selected;
}

const PerceptionSafetyDecision& PerceptionSafetyPolicy::Evaluate(
    const std::vector<SafetyTarget>& targets, const Position3D& robot_position,
    const std::vector<robot_navigation::MapZone>& danger_zones,
    const std::int64_t update_timestamp_ns) {
  bool malformed_person_input = false;
  auto candidate = Classify(
      targets, robot_position, danger_zones, update_timestamp_ns,
      &malformed_person_input);
  if (!candidate.input_valid ||
      (malformed_person_input &&
       Restrictiveness(candidate.level) <= Restrictiveness(decision_.level))) {
    decision_.input_valid = false;
    return decision_;
  }
  candidate.input_valid = true;

  const int candidate_level = Restrictiveness(candidate.level);
  const int current_level = Restrictiveness(decision_.level);
  if (candidate_level >= current_level) {
    decision_ = candidate;
    less_restrictive_observations_ = 0U;
    return decision_;
  }

  ++less_restrictive_observations_;
  if (less_restrictive_observations_ >= config_.clear_observations) {
    decision_ = candidate;
    less_restrictive_observations_ = 0U;
  } else {
    decision_.input_valid = true;
  }
  return decision_;
}

}  // namespace robot_perception
