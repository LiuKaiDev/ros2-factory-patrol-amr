#include "robot_perception/target_manager.hpp"

#include "gtest/gtest.h"

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace robot_perception {
namespace {

constexpr std::int64_t kSecond = 1000000000LL;

TargetObservation Observation(const double x, const double y = 0.0, const double z = 1.0,
                              const std::string& class_name = "person",
                              const std::int64_t timestamp_ns = 1, const double confidence = 0.9) {
    TargetObservation observation;
    observation.class_name = class_name;
    observation.confidence = confidence;
    observation.position = {x, y, z};
    observation.timestamp_ns = timestamp_ns;
    observation.depth_valid = true;
    return observation;
}

TargetManagerConfig Config(const std::size_t confirm_frames = 3U,
                           const std::size_t lost_frames = 5U,
                           const double max_match_distance = 0.5, const double ema_alpha = 0.4,
                           const double processed_cooldown_sec = 10.0) {
    return {confirm_frames, lost_frames, max_match_distance, ema_alpha, processed_cooldown_sec};
}

TEST(TargetManagerTest, NewObservationCreatesTentativeTarget) {
    TargetManager manager;
    const auto& targets = manager.Update({Observation(1.0)}, 1);

    ASSERT_EQ(targets.size(), 1U);
    EXPECT_EQ(targets[0].target_id, 1U);
    EXPECT_EQ(targets[0].state, TrackingState::kTentative);
    EXPECT_EQ(targets[0].hit_count, 1U);
}

TEST(TargetManagerTest, ConfirmFramesPromotesTarget) {
    TargetManager manager(Config(3U));
    manager.Update({Observation(1.0, 0.0, 1.0, "person", 1)}, 1);
    manager.Update({Observation(1.1, 0.0, 1.0, "person", 2)}, 2);
    const auto& targets = manager.Update({Observation(1.05, 0.0, 1.0, "person", 3)}, 3);

    ASSERT_EQ(targets.size(), 1U);
    EXPECT_EQ(targets[0].state, TrackingState::kConfirmed);
    EXPECT_EQ(targets[0].hit_count, 3U);
}

TEST(TargetManagerTest, TargetIdRemainsStableAcrossMatches) {
    TargetManager manager;
    manager.Update({Observation(0.0)}, 1);
    const std::uint32_t target_id = manager.targets()[0].target_id;
    manager.Update({Observation(0.2, 0.0, 1.0, "person", 2)}, 2);

    ASSERT_EQ(manager.targets().size(), 1U);
    EXPECT_EQ(manager.targets()[0].target_id, target_id);
}

TEST(TargetManagerTest, SameClassNearbyObservationAssociates) {
    TargetManager manager(Config(3U, 5U, 0.5));
    manager.Update({Observation(0.0)}, 1);
    const auto& targets = manager.Update({Observation(0.49, 0.0, 1.0, "person", 2)}, 2);

    ASSERT_EQ(targets.size(), 1U);
    EXPECT_EQ(targets[0].hit_count, 2U);
}

TEST(TargetManagerTest, DistantObservationCreatesAnotherTarget) {
    TargetManager manager(Config(3U, 5U, 0.5));
    manager.Update({Observation(0.0)}, 1);
    const auto& targets = manager.Update({Observation(0.51, 0.0, 1.0, "person", 2)}, 2);

    ASSERT_EQ(targets.size(), 2U);
    EXPECT_NE(targets[0].target_id, targets[1].target_id);
}

TEST(TargetManagerTest, DifferentClassDoesNotAssociate) {
    TargetManager manager;
    manager.Update({Observation(0.0)}, 1);
    const auto& targets = manager.Update({Observation(0.0, 0.0, 1.0, "fire_extinguisher", 2)}, 2);

    ASSERT_EQ(targets.size(), 2U);
    EXPECT_NE(targets[0].class_name, targets[1].class_name);
}

TEST(TargetManagerTest, OneFrameDropoutPreservesTarget) {
    TargetManager manager(Config(1U, 3U));
    manager.Update({Observation(0.0)}, 1);
    const std::uint32_t target_id = manager.targets()[0].target_id;
    manager.Update({}, 2);
    const auto& targets = manager.Update({Observation(0.1, 0.0, 1.0, "person", 3)}, 3);

    ASSERT_EQ(targets.size(), 1U);
    EXPECT_EQ(targets[0].target_id, target_id);
    EXPECT_EQ(targets[0].state, TrackingState::kConfirmed);
    EXPECT_EQ(targets[0].missed_frames, 0U);
}

TEST(TargetManagerTest, LostFramesTransitionsToLost) {
    TargetManager manager(Config(1U, 3U));
    manager.Update({Observation(0.0)}, 1);
    manager.Update({}, 2);
    manager.Update({}, 3);
    EXPECT_EQ(manager.targets()[0].state, TrackingState::kConfirmed);
    manager.Update({}, 4);

    ASSERT_EQ(manager.targets().size(), 1U);
    EXPECT_EQ(manager.targets()[0].state, TrackingState::kLost);
}

TEST(TargetManagerTest, LostTargetReacquiresSameIdWithinRetentionWindow) {
    TargetManager manager(Config(1U, 2U));
    manager.Update({Observation(0.0)}, 1);
    const std::uint32_t target_id = manager.targets()[0].target_id;
    manager.Update({}, 2);
    manager.Update({}, 3);
    ASSERT_EQ(manager.targets()[0].state, TrackingState::kLost);

    const auto& targets = manager.Update({Observation(0.1, 0.0, 1.0, "person", 4)}, 4);
    ASSERT_EQ(targets.size(), 1U);
    EXPECT_EQ(targets[0].target_id, target_id);
    EXPECT_EQ(targets[0].state, TrackingState::kConfirmed);
}

TEST(TargetManagerTest, LostTargetIsEventuallyRetired) {
    TargetManager manager(Config(1U, 2U));
    manager.Update({Observation(0.0)}, 1);
    for (std::int64_t timestamp = 2; timestamp <= 6; ++timestamp) {
        manager.Update({}, timestamp);
    }

    EXPECT_TRUE(manager.targets().empty());
}

TEST(TargetManagerTest, NearbyDuplicatesInOneCycleCreateOneTarget) {
    TargetManager manager;
    const auto& targets =
        manager.Update({Observation(0.0), Observation(0.1, 0.0, 1.0, "person", 1)}, 1);

    ASSERT_EQ(targets.size(), 1U);
    EXPECT_EQ(targets[0].hit_count, 1U);
}

TEST(TargetManagerTest, SupportsMultipleSimultaneousTargets) {
    TargetManager manager;
    const auto& targets = manager.Update(
        {Observation(0.0), Observation(2.0), Observation(4.0, 0.0, 1.0, "fire_extinguisher")}, 1);

    ASSERT_EQ(targets.size(), 3U);
    EXPECT_EQ(targets[0].target_id, 1U);
    EXPECT_EQ(targets[1].target_id, 2U);
    EXPECT_EQ(targets[2].target_id, 3U);
}

TEST(TargetManagerTest, OneObservationCannotUpdateTwoTargets) {
    TargetManager manager(Config(3U, 5U, 0.5));
    manager.Update({Observation(0.0), Observation(0.8)}, 1);
    ASSERT_EQ(manager.targets().size(), 2U);

    const auto& targets = manager.Update({Observation(0.4, 0.0, 1.0, "person", 2)}, 2);
    ASSERT_EQ(targets.size(), 2U);
    EXPECT_EQ(targets[0].hit_count + targets[1].hit_count, 3U);
    EXPECT_EQ(targets[0].missed_frames + targets[1].missed_frames, 1U);
}

TEST(TargetManagerTest, OneTargetCannotReceiveTwoObservationsInOneCycle) {
    TargetManager manager;
    manager.Update({Observation(0.0)}, 1);
    const auto& targets = manager.Update(
        {Observation(0.1, 0.0, 1.0, "person", 2), Observation(0.2, 0.0, 1.0, "person", 2)}, 2);

    ASSERT_EQ(targets.size(), 1U);
    EXPECT_EQ(targets[0].hit_count, 2U);
}

TEST(TargetManagerTest, EmaSmoothsPositionAndPreservesRawPosition) {
    TargetManager manager(Config(3U, 5U, 3.0, 0.4));
    manager.Update({Observation(0.0, 0.0, 0.0)}, 1);
    const auto& targets = manager.Update({Observation(1.0, -1.0, 2.0, "person", 2)}, 2);

    ASSERT_EQ(targets.size(), 1U);
    EXPECT_DOUBLE_EQ(targets[0].raw_position.x, 1.0);
    EXPECT_DOUBLE_EQ(targets[0].raw_position.y, -1.0);
    EXPECT_DOUBLE_EQ(targets[0].raw_position.z, 2.0);
    EXPECT_NEAR(targets[0].filtered_position.x, 0.4, 1.0e-12);
    EXPECT_NEAR(targets[0].filtered_position.y, -0.4, 1.0e-12);
    EXPECT_NEAR(targets[0].filtered_position.z, 0.8, 1.0e-12);
}

TEST(TargetManagerTest, RejectsInvalidEmaAlpha) {
    EXPECT_THROW(TargetManager(Config(3U, 5U, 0.5, 0.0)), std::invalid_argument);
    EXPECT_THROW(TargetManager(Config(3U, 5U, 0.5, 1.01)), std::invalid_argument);
    EXPECT_THROW(TargetManager(Config(3U, 5U, 0.5, std::numeric_limits<double>::quiet_NaN())),
                 std::invalid_argument);
}

TEST(TargetManagerTest, RejectsInvalidMaxMatchDistance) {
    EXPECT_THROW(TargetManager(Config(3U, 5U, 0.0)), std::invalid_argument);
    EXPECT_THROW(TargetManager(Config(3U, 5U, -0.1)), std::invalid_argument);
    EXPECT_THROW(TargetManager(Config(3U, 5U, std::numeric_limits<double>::infinity())),
                 std::invalid_argument);
}

TEST(TargetManagerTest, RejectsOtherInvalidConfiguration) {
    EXPECT_THROW(TargetManager(Config(0U)), std::invalid_argument);
    EXPECT_THROW(TargetManager(Config(3U, 0U)), std::invalid_argument);
    EXPECT_THROW(TargetManager(Config(3U, 5U, 0.5, 0.4, -1.0)), std::invalid_argument);
}

TEST(TargetManagerTest, MarkProcessedChangesLifecycleState) {
    TargetManager manager(Config(1U));
    manager.Update({Observation(0.0)}, 1);

    ASSERT_TRUE(manager.MarkProcessed(1U, 2));
    EXPECT_EQ(manager.targets()[0].state, TrackingState::kProcessed);
    EXPECT_FALSE(manager.MarkProcessed(999U, 2));
}

TEST(TargetManagerTest, ProcessedCooldownSuppressesNewActionableTarget) {
    TargetManager manager(Config(1U, 5U, 0.5, 0.4, 10.0));
    manager.Update({Observation(0.0, 0.0, 1.0, "person", kSecond)}, kSecond);
    ASSERT_TRUE(manager.MarkProcessed(1U, 2 * kSecond));

    const auto& targets =
        manager.Update({Observation(0.1, 0.0, 1.0, "person", 5 * kSecond)}, 5 * kSecond);
    ASSERT_EQ(targets.size(), 1U);
    EXPECT_EQ(targets[0].target_id, 1U);
    EXPECT_EQ(targets[0].state, TrackingState::kProcessed);
}

TEST(TargetManagerTest, CooldownExpiryReactivatesSameIdAsTentative) {
    TargetManager manager(Config(3U, 5U, 0.5, 0.4, 10.0));
    manager.Update({Observation(0.0, 0.0, 1.0, "person", kSecond)}, kSecond);
    ASSERT_TRUE(manager.MarkProcessed(1U, 2 * kSecond));

    const auto& targets =
        manager.Update({Observation(0.1, 0.0, 1.0, "person", 12 * kSecond)}, 12 * kSecond);
    ASSERT_EQ(targets.size(), 1U);
    EXPECT_EQ(targets[0].target_id, 1U);
    EXPECT_EQ(targets[0].state, TrackingState::kTentative);
    EXPECT_EQ(targets[0].hit_count, 1U);
}

TEST(TargetManagerTest, RejectsInvalidAndNonFiniteObservations) {
    TargetManager manager;
    auto nan_position = Observation(0.0);
    nan_position.position.x = std::numeric_limits<double>::quiet_NaN();
    auto infinite_position = Observation(0.0);
    infinite_position.position.z = std::numeric_limits<double>::infinity();
    auto invalid_depth = Observation(0.0);
    invalid_depth.depth_valid = false;
    auto malformed_class = Observation(0.0);
    malformed_class.class_name.clear();
    auto invalid_confidence = Observation(0.0);
    invalid_confidence.confidence = 1.1;

    manager.Update(
        {nan_position, infinite_position, invalid_depth, malformed_class, invalid_confidence}, 1);
    EXPECT_TRUE(manager.targets().empty());
}

TEST(TargetManagerTest, RejectsOutOfOrderUpdateWithoutMutatingState) {
    TargetManager manager;
    manager.Update({Observation(0.0, 0.0, 1.0, "person", 10)}, 10);
    const auto& targets = manager.Update({Observation(5.0, 0.0, 1.0, "person", 9)}, 9);

    ASSERT_EQ(targets.size(), 1U);
    EXPECT_DOUBLE_EQ(targets[0].raw_position.x, 0.0);
    EXPECT_EQ(targets[0].hit_count, 1U);
}

}  // namespace
}  // namespace robot_perception
