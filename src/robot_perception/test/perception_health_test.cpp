#include <gtest/gtest.h>

#include "robot_perception/perception_health.hpp"

TEST(PerceptionHealth, StreamStartupAndStaleTransitions)
{
  robot_perception::HealthThresholds thresholds;
  thresholds.stale_warn_sec = 0.5;
  thresholds.stale_error_sec = 1.0;
  EXPECT_EQ(robot_perception::EvaluateStreamHealth(false, 0.0, thresholds).level, 1U);
  EXPECT_EQ(robot_perception::EvaluateStreamHealth(true, 0.1, thresholds).level, 0U);
  EXPECT_EQ(robot_perception::EvaluateStreamHealth(true, 0.75, thresholds).level, 1U);
  EXPECT_EQ(robot_perception::EvaluateStreamHealth(true, 1.5, thresholds).level, 2U);
}

TEST(PerceptionHealth, InvalidDepthRatio)
{
  EXPECT_EQ(robot_perception::EvaluateInvalidRatio(true, 0.1, 0.25, 0.6).level, 0U);
  EXPECT_EQ(robot_perception::EvaluateInvalidRatio(true, 0.3, 0.25, 0.6).level, 1U);
  EXPECT_EQ(robot_perception::EvaluateInvalidRatio(true, 0.8, 0.25, 0.6).level, 2U);
}

TEST(PerceptionHealth, DetectorFaultsAndRecovery)
{
  EXPECT_EQ(
    robot_perception::EvaluateDetectorHealth(false, 0, 0.0, 800.0, 1500.0, false, 0.0, 3.0).level,
    2U);
  EXPECT_EQ(
    robot_perception::EvaluateDetectorHealth(true, 3, 100.0, 800.0, 1500.0, true, 0.0, 3.0).level,
    2U);
  EXPECT_EQ(
    robot_perception::EvaluateDetectorHealth(true, 0, 900.0, 800.0, 1500.0, true, 0.0, 3.0).level,
    1U);
  EXPECT_EQ(
    robot_perception::EvaluateDetectorHealth(true, 0, 100.0, 800.0, 1500.0, true, 0.0, 3.0).level,
    0U);
}

TEST(PerceptionHealth, TfFailuresAndAggregation)
{
  EXPECT_EQ(robot_perception::EvaluateTfHealth(true, 2, 2, 5).level, 1U);
  EXPECT_EQ(robot_perception::EvaluateTfHealth(true, 5, 2, 5).level, 2U);
  EXPECT_EQ(robot_perception::EvaluateTfHealth(true, 0, 2, 5).level, 0U);
  EXPECT_EQ(robot_perception::MaxHealthLevel(1U, 2U), 2U);
  EXPECT_EQ(robot_perception::MaxHealthLevel(0U, 1U), 1U);
}

TEST(PerceptionHealth, SafetyRequiresAValidProjectedObservation)
{
  EXPECT_FALSE(robot_perception::HasValidSafetyObservation(0U, 0U));
  EXPECT_FALSE(robot_perception::HasValidSafetyObservation(1U, 0U));
  EXPECT_TRUE(robot_perception::HasValidSafetyObservation(1U, 1U));
}
