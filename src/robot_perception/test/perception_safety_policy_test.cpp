#include "robot_perception/perception_safety_policy.hpp"

#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

namespace robot_perception {
namespace {

constexpr std::int64_t kNowNs = 10'000'000'000LL;

SafetyTarget Person(
    const double x, const double y = 0.0,
    const TrackingState state = TrackingState::kConfirmed,
    const std::int64_t stamp_ns = kNowNs) {
  SafetyTarget target;
  target.target_id = 7U;
  target.class_name = "person";
  target.position = {x, y, 0.8};
  target.state = state;
  target.last_observation_ns = stamp_ns;
  target.depth_valid = true;
  return target;
}

robot_navigation::MapZone DangerZone() {
  robot_navigation::MapZone zone;
  zone.zone_id = "fixture_danger_zone";
  zone.map_name = "factory_patrol";
  zone.type = "danger_zone";
  zone.frame_id = "map";
  zone.enabled = true;
  zone.polygon_x = {2.0, 3.0, 3.0, 2.0};
  zone.polygon_y = {-1.0, -1.0, 0.0, 0.0};
  return zone;
}

TEST(PerceptionSafetyPolicyTest, NoPersonIsClear) {
  PerceptionSafetyPolicy policy;
  EXPECT_EQ(policy.Evaluate({}, {}, {}, kNowNs).level,
            PerceptionSafetyLevel::kClear);
}

TEST(PerceptionSafetyPolicyTest, FarPersonIsClear) {
  PerceptionSafetyPolicy policy;
  EXPECT_EQ(policy.Evaluate({Person(3.01)}, {}, {}, kNowNs).level,
            PerceptionSafetyLevel::kClear);
}

TEST(PerceptionSafetyPolicyTest, PersonInSlowRangeIsSpeedLimited) {
  PerceptionSafetyPolicy policy;
  const auto& decision = policy.Evaluate({Person(2.0)}, {}, {}, kNowNs);
  EXPECT_EQ(decision.level, PerceptionSafetyLevel::kSpeedLimited);
  EXPECT_EQ(decision.reason, PerceptionSafetyReason::kPersonNear);
  EXPECT_DOUBLE_EQ(decision.distance_m, 2.0);
}

TEST(PerceptionSafetyPolicyTest, PersonInsideStopRangeStops) {
  PerceptionSafetyPolicy policy;
  const auto& decision = policy.Evaluate({Person(1.49)}, {}, {}, kNowNs);
  EXPECT_EQ(decision.level, PerceptionSafetyLevel::kStop);
  EXPECT_EQ(decision.reason, PerceptionSafetyReason::kPersonTooClose);
}

TEST(PerceptionSafetyPolicyTest, ExactThresholdsFollowDocumentedBoundaries) {
  PerceptionSafetyPolicy at_stop;
  EXPECT_EQ(at_stop.Evaluate({Person(1.5)}, {}, {}, kNowNs).level,
            PerceptionSafetyLevel::kSpeedLimited);
  PerceptionSafetyPolicy at_slow;
  EXPECT_EQ(at_slow.Evaluate({Person(3.0)}, {}, {}, kNowNs).level,
            PerceptionSafetyLevel::kSpeedLimited);
}

TEST(PerceptionSafetyPolicyTest, NonFinitePersonDoesNotCreateFalseClear) {
  PerceptionSafetyPolicy policy;
  policy.Evaluate({Person(1.0)}, {}, {}, kNowNs);
  auto invalid = Person(1.0);
  invalid.position.x = std::numeric_limits<double>::quiet_NaN();

  const auto& decision = policy.Evaluate({invalid}, {}, {}, kNowNs);

  EXPECT_EQ(decision.level, PerceptionSafetyLevel::kStop);
  EXPECT_FALSE(decision.input_valid);
}

TEST(PerceptionSafetyPolicyTest, InvalidStopThresholdIsRejected) {
  PerceptionSafetyConfig config;
  config.person_stop_distance = 0.0;
  EXPECT_THROW(PerceptionSafetyPolicy policy(config), std::invalid_argument);
}

TEST(PerceptionSafetyPolicyTest, NonFiniteSlowThresholdIsRejected) {
  PerceptionSafetyConfig config;
  config.person_slow_distance = std::numeric_limits<double>::infinity();
  EXPECT_THROW(PerceptionSafetyPolicy policy(config), std::invalid_argument);
}

TEST(PerceptionSafetyPolicyTest, SlowThresholdNotGreaterThanStopIsRejected) {
  PerceptionSafetyConfig config;
  config.person_stop_distance = 2.0;
  config.person_slow_distance = 2.0;
  EXPECT_THROW(PerceptionSafetyPolicy policy(config), std::invalid_argument);
}

TEST(PerceptionSafetyPolicyTest, MultiplePersonsChooseMostRestrictive) {
  PerceptionSafetyPolicy policy;
  auto far = Person(4.0);
  far.target_id = 1U;
  auto near = Person(2.0);
  near.target_id = 2U;
  auto close = Person(1.0);
  close.target_id = 3U;

  const auto& decision = policy.Evaluate({far, near, close}, {}, {}, kNowNs);

  EXPECT_EQ(decision.level, PerceptionSafetyLevel::kStop);
  EXPECT_EQ(decision.target_id, 3U);
}

TEST(PerceptionSafetyPolicyTest, PersonInDangerZoneStopsBeyondDistanceStop) {
  PerceptionSafetyPolicy policy;
  const auto& decision = policy.Evaluate(
      {Person(2.5, -0.5)}, {}, {DangerZone()}, kNowNs);
  EXPECT_EQ(decision.level, PerceptionSafetyLevel::kStop);
  EXPECT_EQ(decision.reason, PerceptionSafetyReason::kPersonInDangerZone);
  EXPECT_EQ(decision.zone_id, "fixture_danger_zone");
}

TEST(PerceptionSafetyPolicyTest, PersonOutsideDangerZoneUsesDistancePolicy) {
  PerceptionSafetyPolicy policy;
  const auto& decision = policy.Evaluate(
      {Person(2.5, 0.5)}, {}, {DangerZone()}, kNowNs);
  EXPECT_EQ(decision.level, PerceptionSafetyLevel::kSpeedLimited);
  EXPECT_EQ(decision.reason, PerceptionSafetyReason::kPersonNear);
}

TEST(PerceptionSafetyPolicyTest, DangerZoneBoundaryIsInside) {
  PerceptionSafetyPolicy policy;
  EXPECT_EQ(
      policy.Evaluate({Person(2.0, -0.5)}, {}, {DangerZone()}, kNowNs).reason,
      PerceptionSafetyReason::kPersonInDangerZone);
}

TEST(PerceptionSafetyPolicyTest, ShortDropoutRetainsStop) {
  PerceptionSafetyPolicy policy;
  policy.Evaluate({Person(1.0)}, {}, {}, kNowNs);
  EXPECT_EQ(policy.Evaluate({}, {}, {}, kNowNs + 1).level,
            PerceptionSafetyLevel::kStop);
}

TEST(PerceptionSafetyPolicyTest, SufficientClearObservationsRecover) {
  PerceptionSafetyPolicy policy;
  policy.Evaluate({Person(1.0)}, {}, {}, kNowNs);
  policy.Evaluate({}, {}, {}, kNowNs + 1);
  policy.Evaluate({}, {}, {}, kNowNs + 2);
  EXPECT_EQ(policy.Evaluate({}, {}, {}, kNowNs + 3).level,
            PerceptionSafetyLevel::kClear);
}

TEST(PerceptionSafetyPolicyTest, StopHysteresisHoldsAtOnePointSixMeters) {
  PerceptionSafetyPolicy policy;
  policy.Evaluate({Person(1.0)}, {}, {}, kNowNs);
  for (int index = 1; index <= 4; ++index) {
    EXPECT_EQ(policy.Evaluate({Person(1.6)}, {}, {}, kNowNs + index).level,
              PerceptionSafetyLevel::kStop);
  }
}

TEST(PerceptionSafetyPolicyTest, SlowHysteresisHoldsAtThreePointOneMeters) {
  PerceptionSafetyPolicy policy;
  policy.Evaluate({Person(2.0)}, {}, {}, kNowNs);
  for (int index = 1; index <= 4; ++index) {
    EXPECT_EQ(policy.Evaluate({Person(3.1)}, {}, {}, kNowNs + index).level,
              PerceptionSafetyLevel::kSpeedLimited);
  }
}

TEST(PerceptionSafetyPolicyTest, LostTargetIsNotEligibleAndRecovers) {
  PerceptionSafetyPolicy policy;
  policy.Evaluate({Person(1.0)}, {}, {}, kNowNs);
  auto lost = Person(1.0, 0.0, TrackingState::kLost);
  lost.missed_frames = 5U;
  policy.Evaluate({lost}, {}, {}, kNowNs + 1);
  policy.Evaluate({lost}, {}, {}, kNowNs + 2);
  EXPECT_EQ(policy.Evaluate({lost}, {}, {}, kNowNs + 3).level,
            PerceptionSafetyLevel::kClear);
}

TEST(PerceptionSafetyPolicyTest, NonPersonTargetDoesNotAffectSafety) {
  PerceptionSafetyPolicy policy;
  auto chair = Person(0.5);
  chair.class_name = "chair";
  EXPECT_EQ(policy.Evaluate({chair}, {}, {}, kNowNs).level,
            PerceptionSafetyLevel::kClear);
}

TEST(PerceptionSafetyPolicyTest, StaleTargetDoesNotRemainActiveIndefinitely) {
  PerceptionSafetyConfig config;
  config.clear_observations = 1U;
  config.max_target_age_sec = 1.0;
  PerceptionSafetyPolicy policy(config);
  EXPECT_EQ(policy.Evaluate(
                {Person(1.0, 0.0, TrackingState::kConfirmed, kNowNs - 2'000'000'000LL)},
                {}, {}, kNowNs)
                .level,
            PerceptionSafetyLevel::kClear);
}

TEST(PerceptionSafetyPolicyTest, TentativeTargetIsNotEligible) {
  PerceptionSafetyPolicy policy;
  EXPECT_EQ(policy.Evaluate(
                {Person(0.5, 0.0, TrackingState::kTentative)}, {}, {}, kNowNs)
                .level,
            PerceptionSafetyLevel::kClear);
}

TEST(PerceptionSafetyPolicyTest, ProcessedPersonRemainsEligibleWhileObserved) {
  PerceptionSafetyPolicy policy;
  EXPECT_EQ(policy.Evaluate(
                {Person(1.0, 0.0, TrackingState::kProcessed)}, {}, {}, kNowNs)
                .level,
            PerceptionSafetyLevel::kStop);
}

TEST(PerceptionSafetyPolicyTest, InvalidRobotPoseCannotClearExistingStop) {
  PerceptionSafetyPolicy policy;
  policy.Evaluate({Person(1.0)}, {}, {}, kNowNs);
  Position3D invalid_robot;
  invalid_robot.x = std::numeric_limits<double>::quiet_NaN();
  const auto& decision = policy.Evaluate({}, invalid_robot, {}, kNowNs + 1);
  EXPECT_EQ(decision.level, PerceptionSafetyLevel::kStop);
  EXPECT_FALSE(decision.input_valid);
}

TEST(PerceptionSafetyPolicyTest, DisabledPolicyAlwaysClears) {
  PerceptionSafetyConfig config;
  config.enabled = false;
  PerceptionSafetyPolicy policy(config);
  EXPECT_EQ(policy.Evaluate({Person(0.5)}, {}, {DangerZone()}, kNowNs).level,
            PerceptionSafetyLevel::kClear);
}

}  // namespace
}  // namespace robot_perception
