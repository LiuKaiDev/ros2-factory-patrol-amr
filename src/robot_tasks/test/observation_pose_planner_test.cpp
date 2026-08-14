#include <cmath>
#include <limits>

#include "gtest/gtest.h"
#include "robot_tasks/observation_pose_planner.hpp"

namespace robot_tasks {
namespace {

geometry_msgs::msg::PoseStamped Pose(
    const double x, const double y, const double yaw = 0.0) {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.z = std::sin(yaw * 0.5);
  pose.pose.orientation.w = std::cos(yaw * 0.5);
  return pose;
}

double DistanceToTarget(
    const geometry_msgs::msg::PoseStamped& observation,
    const geometry_msgs::msg::PoseStamped& target) {
  return std::hypot(observation.pose.position.x - target.pose.position.x,
                    observation.pose.position.y - target.pose.position.y);
}

double Yaw(const geometry_msgs::msg::PoseStamped& pose) {
  return std::atan2(
      2.0 * pose.pose.orientation.w * pose.pose.orientation.z,
      1.0 - 2.0 * pose.pose.orientation.z * pose.pose.orientation.z);
}

void ExpectFacesTarget(
    const geometry_msgs::msg::PoseStamped& observation,
    const geometry_msgs::msg::PoseStamped& target) {
  const double expected = std::atan2(
      target.pose.position.y - observation.pose.position.y,
      target.pose.position.x - observation.pose.position.x);
  EXPECT_NEAR(std::remainder(Yaw(observation) - expected, 2.0 * M_PI), 0.0,
              1.0e-9);
}

TEST(ObservationPosePlannerTest, TargetInFrontUsesRobotSideStandoff) {
  const auto target = Pose(3.0, 0.0);
  const auto result = ComputeObservationPose(target, Pose(0.0, 0.0), 1.2);
  ASSERT_TRUE(result.pose);
  EXPECT_NEAR(result.pose->pose.position.x, 1.8, 1.0e-9);
  EXPECT_NEAR(result.pose->pose.position.y, 0.0, 1.0e-9);
  ExpectFacesTarget(*result.pose, target);
}

TEST(ObservationPosePlannerTest, TargetBehindUsesRobotSideStandoff) {
  const auto target = Pose(-3.0, 0.0);
  const auto result = ComputeObservationPose(target, Pose(0.0, 0.0), 1.2);
  ASSERT_TRUE(result.pose);
  EXPECT_NEAR(result.pose->pose.position.x, -1.8, 1.0e-9);
  ExpectFacesTarget(*result.pose, target);
}

TEST(ObservationPosePlannerTest, HandlesLeftAndRightTargets) {
  for (const double target_y : {-2.0, 2.0}) {
    const auto target = Pose(0.0, target_y);
    const auto result = ComputeObservationPose(target, Pose(0.0, 0.0), 1.2);
    ASSERT_TRUE(result.pose);
    EXPECT_NEAR(DistanceToTarget(*result.pose, target), 1.2, 1.0e-9);
    ExpectFacesTarget(*result.pose, target);
  }
}

TEST(ObservationPosePlannerTest, RespectsConfiguredStandoffAndReturnsUnitQuaternion) {
  const auto target = Pose(2.5, 1.5);
  const auto result = ComputeObservationPose(target, Pose(-0.5, -0.25), 1.35);
  ASSERT_TRUE(result.pose);
  EXPECT_NEAR(DistanceToTarget(*result.pose, target), 1.35, 1.0e-9);
  const auto& q = result.pose->pose.orientation;
  EXPECT_NEAR(std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w)), 1.0,
              1.0e-9);
  EXPECT_TRUE(IsFiniteValidPose(*result.pose));
  ExpectFacesTarget(*result.pose, target);
}

TEST(ObservationPosePlannerTest, DegeneratePositionUsesOppositeRobotHeading) {
  const auto target = Pose(1.0, 1.0);
  const auto result = ComputeObservationPose(target, Pose(1.0, 1.0, M_PI_2), 1.2);
  ASSERT_TRUE(result.pose);
  EXPECT_NEAR(result.pose->pose.position.x, 1.0, 1.0e-9);
  EXPECT_NEAR(result.pose->pose.position.y, -0.2, 1.0e-9);
  EXPECT_NEAR(DistanceToTarget(*result.pose, target), 1.2, 1.0e-9);
  ExpectFacesTarget(*result.pose, target);
}

TEST(ObservationPosePlannerTest, RejectsNonfiniteTarget) {
  auto target = Pose(2.0, 0.0);
  target.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(ComputeObservationPose(target, Pose(0.0, 0.0), 1.2).pose);
}

TEST(ObservationPosePlannerTest, RejectsInvalidStandoff) {
  EXPECT_FALSE(ComputeObservationPose(Pose(2.0, 0.0), Pose(0.0, 0.0), 0.0).pose);
  EXPECT_FALSE(ComputeObservationPose(
                   Pose(2.0, 0.0), Pose(0.0, 0.0),
                   std::numeric_limits<double>::infinity())
                   .pose);
}

TEST(ObservationPosePlannerTest, RejectsWrongFrame) {
  auto target = Pose(2.0, 0.0);
  target.header.frame_id = "camera_color_optical_frame";
  EXPECT_FALSE(ComputeObservationPose(target, Pose(0.0, 0.0), 1.2).pose);
}

}  // namespace
}  // namespace robot_tasks
