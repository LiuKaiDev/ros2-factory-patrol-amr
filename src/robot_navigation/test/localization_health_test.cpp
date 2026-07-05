#include "robot_navigation/localization_health.hpp"

#include <array>

#include <gtest/gtest.h>

namespace robot_navigation {

TEST(LocalizationHealthTest, SmallCovarianceIsOk) {
  LocalizationHealthThresholds thresholds;
  LocalizationCovariance covariance;
  covariance.x = 0.02;
  covariance.y = 0.03;
  covariance.yaw = 0.04;

  EXPECT_EQ(EvaluateCovariance(covariance, thresholds), LocalizationSignalLevel::kOk);
}

TEST(LocalizationHealthTest, WarnCovarianceIsUnstableCondition) {
  LocalizationHealthThresholds thresholds;
  LocalizationCovariance covariance;
  covariance.x = 0.26;
  covariance.y = 0.03;
  covariance.yaw = 0.04;

  EXPECT_EQ(EvaluateCovariance(covariance, thresholds), LocalizationSignalLevel::kWarn);
}

TEST(LocalizationHealthTest, LostCovarianceIsLostCondition) {
  LocalizationHealthThresholds thresholds;
  LocalizationCovariance covariance;
  covariance.x = 0.02;
  covariance.y = 0.81;
  covariance.yaw = 0.04;

  EXPECT_EQ(EvaluateCovariance(covariance, thresholds), LocalizationSignalLevel::kLost);
}

TEST(LocalizationHealthTest, YawCovarianceCanTriggerLostByItself) {
  LocalizationHealthThresholds thresholds;
  LocalizationCovariance covariance;
  covariance.x = 0.02;
  covariance.y = 0.03;
  covariance.yaw = 0.9;

  EXPECT_EQ(EvaluateCovariance(covariance, thresholds), LocalizationSignalLevel::kLost);
}

TEST(LocalizationHealthTest, ExtractsXYYawCovariance) {
  std::array<double, 36> covariance {};
  covariance[0] = 0.1;
  covariance[7] = 0.2;
  covariance[35] = 0.3;

  const auto result = ExtractLocalizationCovariance(covariance);

  EXPECT_DOUBLE_EQ(result.x, 0.1);
  EXPECT_DOUBLE_EQ(result.y, 0.2);
  EXPECT_DOUBLE_EQ(result.yaw, 0.3);
  EXPECT_DOUBLE_EQ(MaxPlanarCovariance(result), 0.2);
}

TEST(LocalizationHealthTest, StateStringConversionRoundTripsKnownStates) {
  EXPECT_EQ(ToString(LocalizationHealthState::kUnknown), "LOCALIZATION_UNKNOWN");
  EXPECT_EQ(ToString(LocalizationHealthState::kOk), "LOCALIZATION_OK");
  EXPECT_EQ(ToString(LocalizationHealthState::kUnstable), "LOCALIZATION_UNSTABLE");
  EXPECT_EQ(ToString(LocalizationHealthState::kLost), "LOCALIZATION_LOST");
  EXPECT_EQ(ToString(LocalizationHealthState::kRecovering), "LOCALIZATION_RECOVERING");
  EXPECT_EQ(ToString(LocalizationHealthState::kRecovered), "LOCALIZATION_RECOVERED");
  EXPECT_EQ(
      LocalizationHealthStateFromString("LOCALIZATION_RECOVERED"),
      LocalizationHealthState::kRecovered);
  EXPECT_EQ(
      LocalizationHealthStateFromString("something_else"),
      LocalizationHealthState::kUnknown);
}

TEST(LocalizationHealthTest, TimeoutRequiresSampleAndAgeBeyondThreshold) {
  EXPECT_TRUE(IsTimedOut(false, 0.0, 1.0));
  EXPECT_FALSE(IsTimedOut(true, 0.5, 1.0));
  EXPECT_TRUE(IsTimedOut(true, 1.1, 1.0));
}

}  // namespace robot_navigation
