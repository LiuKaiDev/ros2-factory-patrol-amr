#include <limits>
#include <stdexcept>

#include "gtest/gtest.h"
#include "robot_perception/inspection_event_policy.hpp"

namespace robot_perception {
namespace {

ManagedTarget Target(
    const TrackingState state = TrackingState::kConfirmed,
    const std::string& class_name = "chair", const double confidence = 0.8) {
  ManagedTarget target;
  target.target_id = 7U;
  target.class_name = class_name;
  target.confidence = confidence;
  target.filtered_position = {2.0, 0.5, 0.4};
  target.depth_valid = true;
  target.state = state;
  return target;
}

TEST(InspectionEventPolicyTest, TentativeTargetDoesNotEmit) {
  InspectionEventPolicy policy;
  EXPECT_FALSE(policy.ShouldEmit(Target(TrackingState::kTentative)));
}

TEST(InspectionEventPolicyTest, ConfirmedEligibleTargetEmitsOnce) {
  InspectionEventPolicy policy;
  EXPECT_TRUE(policy.ShouldEmit(Target()));
  EXPECT_TRUE(policy.HasEmitted(7U));
  EXPECT_FALSE(policy.ShouldEmit(Target()));
}

TEST(InspectionEventPolicyTest, RejectsClassOutsideAllowlist) {
  InspectionEventPolicy policy;
  EXPECT_FALSE(policy.ShouldEmit(Target(TrackingState::kConfirmed, "person")));
}

TEST(InspectionEventPolicyTest, RejectsLowConfidence) {
  InspectionEventPolicy policy;
  EXPECT_FALSE(policy.ShouldEmit(Target(TrackingState::kConfirmed, "chair", 0.49)));
}

TEST(InspectionEventPolicyTest, ProcessedTargetDoesNotEmit) {
  InspectionEventPolicy policy;
  EXPECT_FALSE(policy.ShouldEmit(Target(TrackingState::kProcessed)));
}

TEST(InspectionEventPolicyTest, TentativeTransitionRearmsAfterProcessedCooldown) {
  InspectionEventPolicy policy;
  EXPECT_TRUE(policy.ShouldEmit(Target()));
  EXPECT_FALSE(policy.ShouldEmit(Target(TrackingState::kProcessed)));
  EXPECT_FALSE(policy.ShouldEmit(Target(TrackingState::kTentative)));
  EXPECT_TRUE(policy.ShouldEmit(Target()));
}

TEST(InspectionEventPolicyTest, DisabledPolicyDoesNotEmit) {
  InspectionEventPolicyConfig config;
  config.enabled = false;
  InspectionEventPolicy policy(config);
  EXPECT_FALSE(policy.ShouldEmit(Target()));
}

TEST(InspectionEventPolicyTest, InvalidThresholdIsRejected) {
  InspectionEventPolicyConfig config;
  config.min_confidence = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(InspectionEventPolicy policy(config), std::invalid_argument);
}

}  // namespace
}  // namespace robot_perception
