#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "robot_navigation/zone_catalog.hpp"
#include "robot_perception/target_manager.hpp"

namespace robot_perception {

enum class PerceptionSafetyLevel : std::uint8_t {
  kClear = 0,
  kSpeedLimited = 1,
  kStop = 2,
};

enum class PerceptionSafetyReason : std::uint8_t {
  kClear = 0,
  kPersonNear = 1,
  kPersonTooClose = 2,
  kPersonInDangerZone = 3,
};

struct PerceptionSafetyConfig {
  bool enabled = true;
  std::string person_class = "person";
  double person_slow_distance = 3.0;
  double person_stop_distance = 1.5;
  std::size_t clear_observations = 3U;
  double stop_hysteresis = 0.2;
  double slow_hysteresis = 0.2;
  double max_target_age_sec = 2.5;
};

struct SafetyTarget {
  std::uint32_t target_id = 0U;
  std::string class_name;
  Position3D position;
  TrackingState state = TrackingState::kTentative;
  std::int64_t last_observation_ns = 0;
  std::size_t missed_frames = 0U;
  bool depth_valid = false;
};

struct PerceptionSafetyDecision {
  PerceptionSafetyLevel level = PerceptionSafetyLevel::kClear;
  PerceptionSafetyReason reason = PerceptionSafetyReason::kClear;
  std::uint32_t target_id = 0U;
  std::string class_name;
  Position3D target_position;
  double distance_m = -1.0;
  std::string zone_id;
  bool input_valid = true;
};

class PerceptionSafetyPolicy {
 public:
  explicit PerceptionSafetyPolicy(PerceptionSafetyConfig config = {});

  const PerceptionSafetyDecision& Evaluate(
      const std::vector<SafetyTarget>& targets, const Position3D& robot_position,
      const std::vector<robot_navigation::MapZone>& danger_zones,
      std::int64_t update_timestamp_ns);

  const PerceptionSafetyDecision& decision() const { return decision_; }

 private:
  PerceptionSafetyDecision Classify(
      const std::vector<SafetyTarget>& targets, const Position3D& robot_position,
      const std::vector<robot_navigation::MapZone>& danger_zones,
      std::int64_t update_timestamp_ns, bool* malformed_person_input) const;

  PerceptionSafetyConfig config_;
  PerceptionSafetyDecision decision_;
  std::size_t less_restrictive_observations_ = 0U;
};

const char* PerceptionSafetyLevelName(PerceptionSafetyLevel level);
const char* PerceptionSafetyReasonName(PerceptionSafetyReason reason);

}  // namespace robot_perception
