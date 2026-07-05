#pragma once

#include <array>
#include <string>

namespace robot_navigation {

enum class LocalizationHealthState {
  kUnknown,
  kOk,
  kUnstable,
  kLost,
  kRecovering,
  kRecovered,
};

enum class LocalizationSignalLevel {
  kOk,
  kWarn,
  kLost,
};

struct LocalizationCovariance {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct LocalizationHealthThresholds {
  double covariance_warn_xy = 0.25;
  double covariance_lost_xy = 0.8;
  double covariance_warn_yaw = 0.25;
  double covariance_lost_yaw = 0.8;
};

std::string ToString(LocalizationHealthState state);
LocalizationHealthState LocalizationHealthStateFromString(const std::string& state);
LocalizationCovariance ExtractLocalizationCovariance(const std::array<double, 36>& covariance);
double MaxPlanarCovariance(const LocalizationCovariance& covariance);
LocalizationSignalLevel EvaluateCovariance(
    const LocalizationCovariance& covariance, const LocalizationHealthThresholds& thresholds);
bool IsTimedOut(bool has_sample, double sample_age_sec, double timeout_sec);

}  // namespace robot_navigation
