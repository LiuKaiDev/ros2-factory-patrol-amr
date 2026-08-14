#include "robot_perception/perception_health.hpp"

#include <cmath>

namespace robot_perception {

HealthStatus EvaluateStreamHealth(
  const bool seen, const double age_sec, const HealthThresholds & thresholds)
{
  if (!seen || !std::isfinite(age_sec)) {
    return {1U, "waiting for stream"};
  }
  if (age_sec > thresholds.stale_error_sec) {
    return {2U, "stream stale"};
  }
  if (age_sec > thresholds.stale_warn_sec) {
    return {1U, "stream aging"};
  }
  return {0U, "fresh"};
}

HealthStatus EvaluateInvalidRatio(
  const bool seen, const double ratio, const double warn_ratio, const double error_ratio)
{
  if (!seen || !std::isfinite(ratio)) {
    return {1U, "waiting for depth samples"};
  }
  if (ratio >= error_ratio) {
    return {2U, "global invalid depth ratio high"};
  }
  if (ratio >= warn_ratio) {
    return {1U, "global invalid depth ratio elevated"};
  }
  return {0U, "depth samples valid"};
}

HealthStatus EvaluateDetectorHealth(
  const bool model_available, const int consecutive_failures, const double latency_ms,
  const double warn_latency_ms, const double error_latency_ms, const bool output_seen,
  const double output_age_sec, const double output_timeout_sec)
{
  if (!model_available) {
    return {2U, "model/backend unavailable"};
  }
  if (consecutive_failures >= 3) {
    return {2U, "repeated inference failures"};
  }
  if (latency_ms >= error_latency_ms) {
    return {2U, "inference latency high"};
  }
  if (latency_ms >= warn_latency_ms) {
    return {1U, "inference latency elevated"};
  }
  if (!output_seen || output_age_sec > output_timeout_sec) {
    return {1U, "detector output inactive"};
  }
  return {0U, "detector nominal"};
}

HealthStatus EvaluateTfHealth(
  const bool observed, const int consecutive_failures, const int warn_failure_count,
  const int error_failure_count)
{
  if (!observed) {
    return {1U, "waiting for observations"};
  }
  if (consecutive_failures >= error_failure_count) {
    return {2U, "observation-time TF failures"};
  }
  if (consecutive_failures >= warn_failure_count) {
    return {1U, "observation-time TF intermittently unavailable"};
  }
  return {0U, "observation-time TF available"};
}

std::uint8_t MaxHealthLevel(const std::uint8_t left, const std::uint8_t right)
{
  return std::max(left, right);
}

bool HasValidSafetyObservation(
  const std::size_t detection_count, const std::size_t valid_observation_count)
{
  return detection_count > 0U && valid_observation_count > 0U;
}

}  // namespace robot_perception
