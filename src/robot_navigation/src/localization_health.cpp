#include "robot_navigation/localization_health.hpp"

#include <algorithm>

namespace robot_navigation {

std::string ToString(const LocalizationHealthState state) {
  switch (state) {
    case LocalizationHealthState::kUnknown:
      return "LOCALIZATION_UNKNOWN";
    case LocalizationHealthState::kOk:
      return "LOCALIZATION_OK";
    case LocalizationHealthState::kUnstable:
      return "LOCALIZATION_UNSTABLE";
    case LocalizationHealthState::kLost:
      return "LOCALIZATION_LOST";
    case LocalizationHealthState::kRecovering:
      return "LOCALIZATION_RECOVERING";
    case LocalizationHealthState::kRecovered:
      return "LOCALIZATION_RECOVERED";
  }
  return "LOCALIZATION_UNKNOWN";
}

LocalizationHealthState LocalizationHealthStateFromString(const std::string& state) {
  if (state == "LOCALIZATION_OK") {
    return LocalizationHealthState::kOk;
  }
  if (state == "LOCALIZATION_UNSTABLE") {
    return LocalizationHealthState::kUnstable;
  }
  if (state == "LOCALIZATION_LOST") {
    return LocalizationHealthState::kLost;
  }
  if (state == "LOCALIZATION_RECOVERING") {
    return LocalizationHealthState::kRecovering;
  }
  if (state == "LOCALIZATION_RECOVERED") {
    return LocalizationHealthState::kRecovered;
  }
  return LocalizationHealthState::kUnknown;
}

LocalizationCovariance ExtractLocalizationCovariance(const std::array<double, 36>& covariance) {
  LocalizationCovariance result;
  result.x = covariance[0];
  result.y = covariance[7];
  result.yaw = covariance[35];
  return result;
}

double MaxPlanarCovariance(const LocalizationCovariance& covariance) {
  return std::max(covariance.x, covariance.y);
}

LocalizationSignalLevel EvaluateCovariance(
    const LocalizationCovariance& covariance, const LocalizationHealthThresholds& thresholds) {
  const double xy = MaxPlanarCovariance(covariance);
  if (xy >= thresholds.covariance_lost_xy ||
      covariance.yaw >= thresholds.covariance_lost_yaw) {
    return LocalizationSignalLevel::kLost;
  }
  if (xy >= thresholds.covariance_warn_xy ||
      covariance.yaw >= thresholds.covariance_warn_yaw) {
    return LocalizationSignalLevel::kWarn;
  }
  return LocalizationSignalLevel::kOk;
}

bool IsTimedOut(
    const bool has_sample, const double sample_age_sec, const double timeout_sec) {
  return !has_sample || (timeout_sec >= 0.0 && sample_age_sec > timeout_sec);
}

}  // namespace robot_navigation
