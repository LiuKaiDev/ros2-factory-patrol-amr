#include <limits>

#include "gtest/gtest.h"
#include "robot_tasks/visual_inspection_mission.hpp"

namespace robot_tasks {
namespace {

geometry_msgs::msg::PoseStamped RobotPose() {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.orientation.w = 1.0;
  return pose;
}

robot_interfaces_perception::msg::PerceptionEvent Event(
    const std::string& class_name = "chair", const float confidence = 0.8F) {
  robot_interfaces_perception::msg::PerceptionEvent event;
  event.target_id = 12U;
  event.header.stamp.sec = 10;
  event.event_type =
      robot_interfaces_perception::msg::PerceptionEvent::INSPECTION_REQUIRED;
  event.class_name = class_name;
  event.confidence = confidence;
  event.target_pose.header.frame_id = "map";
  event.target_pose.pose.position.x = 2.5;
  event.target_pose.pose.position.y = 0.5;
  event.target_pose.pose.orientation.w = 1.0;
  return event;
}

TEST(VisualInspectionMissionTest, EligibleEventRequestsOneMission) {
  VisualInspectionMission mission;
  const auto decision = mission.Request(Event(), RobotPose());
  EXPECT_TRUE(decision.accepted);
  EXPECT_TRUE(decision.observation_pose);
  EXPECT_EQ(mission.state(), VisualInspectionState::kRequested);
  EXPECT_EQ(mission.active_target_id(), 12U);
}

TEST(VisualInspectionMissionTest, WrongEventTypeDoesNotRequestMission) {
  VisualInspectionMission mission;
  auto event = Event();
  event.event_type =
      robot_interfaces_perception::msg::PerceptionEvent::TARGET_CONFIRMED;
  EXPECT_FALSE(mission.Request(event, RobotPose()).accepted);
}

TEST(VisualInspectionMissionTest, ClassOutsideAllowlistIsRejected) {
  VisualInspectionMission mission;
  EXPECT_FALSE(mission.Request(Event("person"), RobotPose()).accepted);
}

TEST(VisualInspectionMissionTest, LowConfidenceIsRejected) {
  VisualInspectionMission mission;
  EXPECT_FALSE(mission.Request(Event("chair", 0.49F), RobotPose()).accepted);
}

TEST(VisualInspectionMissionTest, DuplicateTargetDoesNotRetrigger) {
  VisualInspectionMission mission;
  ASSERT_TRUE(mission.Request(Event(), RobotPose()).accepted);
  mission.MarkNavigating();
  EXPECT_FALSE(mission.Finish(true).target_id == 0U);
  EXPECT_FALSE(mission.Request(Event(), RobotPose()).accepted);
}

TEST(VisualInspectionMissionTest, NewerPostCooldownEventCanRetrigger) {
  VisualInspectionMission mission;
  auto event = Event();
  ASSERT_TRUE(mission.Request(event, RobotPose()).accepted);
  mission.MarkNavigating();
  ASSERT_TRUE(mission.Finish(true).publish_completion);
  event.header.stamp.sec = 21;
  EXPECT_TRUE(mission.Request(event, RobotPose()).accepted);
}

TEST(VisualInspectionMissionTest, ActiveMissionSuppressesOtherEvent) {
  VisualInspectionMission mission;
  ASSERT_TRUE(mission.Request(Event(), RobotPose()).accepted);
  auto second = Event();
  second.target_id = 13U;
  EXPECT_FALSE(mission.Request(second, RobotPose()).accepted);
  EXPECT_FALSE(mission.HasSeen(13U));
}

TEST(VisualInspectionMissionTest, SuccessPublishesProcessedCompletion) {
  VisualInspectionMission mission;
  ASSERT_TRUE(mission.Request(Event(), RobotPose()).accepted);
  mission.MarkNavigating();
  const auto outcome = mission.Finish(true);
  EXPECT_TRUE(outcome.publish_completion);
  EXPECT_EQ(outcome.target_id, 12U);
  EXPECT_EQ(mission.state(), VisualInspectionState::kSucceeded);
}

TEST(VisualInspectionMissionTest, FailureDoesNotPublishCompletion) {
  VisualInspectionMission mission;
  ASSERT_TRUE(mission.Request(Event(), RobotPose()).accepted);
  mission.MarkNavigating();
  const auto outcome = mission.Finish(false);
  EXPECT_FALSE(outcome.publish_completion);
  EXPECT_EQ(outcome.target_id, 12U);
  EXPECT_EQ(mission.state(), VisualInspectionState::kFailed);
}

TEST(VisualInspectionMissionTest, MalformedTargetPoseIsRejected) {
  VisualInspectionMission mission;
  auto event = Event();
  event.target_pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(mission.Request(event, RobotPose()).accepted);
}

TEST(VisualInspectionMissionTest, InvalidConfigurationIsRejected) {
  VisualInspectionConfig config;
  config.standoff_distance = -1.0;
  EXPECT_THROW(VisualInspectionMission mission(config), std::invalid_argument);
}

}  // namespace
}  // namespace robot_tasks
