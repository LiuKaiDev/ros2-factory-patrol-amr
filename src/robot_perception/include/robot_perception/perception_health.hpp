#ifndef ROBOT_PERCEPTION__PERCEPTION_HEALTH_HPP_
#define ROBOT_PERCEPTION__PERCEPTION_HEALTH_HPP_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>

namespace robot_perception {

struct HealthThresholds {
  double stale_warn_sec{0.5};
  double stale_error_sec{1.0};
};

struct HealthStatus {
  std::uint8_t level{0};
  std::string message;
};

HealthStatus EvaluateStreamHealth(
  bool seen, double age_sec, const HealthThresholds & thresholds);

HealthStatus EvaluateInvalidRatio(
  bool seen, double ratio, double warn_ratio, double error_ratio);

HealthStatus EvaluateDetectorHealth(
  bool model_available, int consecutive_failures, double latency_ms,
  double warn_latency_ms, double error_latency_ms, bool output_seen,
  double output_age_sec, double output_timeout_sec);

HealthStatus EvaluateTfHealth(
  bool observed, int consecutive_failures, int warn_failure_count,
  int error_failure_count);

std::uint8_t MaxHealthLevel(std::uint8_t left, std::uint8_t right);

bool HasValidSafetyObservation(
  std::size_t detection_count, std::size_t valid_observation_count);

}  // namespace robot_perception

#endif  // ROBOT_PERCEPTION__PERCEPTION_HEALTH_HPP_
