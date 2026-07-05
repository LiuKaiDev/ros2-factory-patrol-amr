#include "robot_teleop/cmd_vel_safety.hpp"

#include <initializer_list>

#include <gtest/gtest.h>

namespace {

TEST(CmdVelSafetyTest, ScalesAngularVelocityWhenLinearSpeedIsScaled) {
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 1.0;
  twist.angular.z = 0.8;

  const auto limited = robot_teleop::ApplyDynamicSpeedLimit(twist, 0.25, 1.0);

  EXPECT_NEAR(limited.linear.x, 0.25, 1e-9);
  EXPECT_NEAR(limited.angular.z, 0.2, 1e-9);
}

TEST(CmdVelSafetyTest, ClampsPureRotationInDynamicSpeedLimitedZone) {
  geometry_msgs::msg::Twist twist;
  twist.angular.z = 1.4;

  const auto limited = robot_teleop::ApplyDynamicSpeedLimit(twist, 0.25, 0.5);

  EXPECT_NEAR(limited.linear.x, 0.0, 1e-9);
  EXPECT_NEAR(limited.angular.z, 0.5, 1e-9);
}

TEST(CmdVelSafetyTest, LeavesTwistUnchangedWhenLimitIsDisabled) {
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 1.0;
  twist.angular.z = 1.4;

  const auto limited = robot_teleop::ApplyDynamicSpeedLimit(twist, 0.0, 0.5);

  EXPECT_NEAR(limited.linear.x, 1.0, 1e-9);
  EXPECT_NEAR(limited.angular.z, 1.4, 1e-9);
}

TEST(CmdVelSafetyTest, EvaluateCmdVelSafetyGate_EmergencyStop_PublishesZero) {
  robot_teleop::CmdVelSafetyGateRequest request;
  request.input_twist.linear.x = 0.4;
  request.input_twist.angular.z = 0.2;
  request.emergency_stop = true;

  const auto decision = robot_teleop::EvaluateCmdVelSafetyGate(request);

  EXPECT_TRUE(decision.stopped());
  EXPECT_EQ(decision.stop_reason, robot_teleop::CmdVelStopReason::kEmergencyStop);
  EXPECT_NEAR(decision.output_twist.linear.x, 0.0, 1e-9);
  EXPECT_NEAR(decision.output_twist.angular.z, 0.0, 1e-9);
}

TEST(CmdVelSafetyTest, EvaluateCmdVelSafetyGate_RuntimeSafetyStop_PublishesZero) {
  robot_teleop::CmdVelSafetyGateRequest request;
  request.input_twist.linear.x = 0.4;
  request.runtime_safety_stop = true;

  const auto decision = robot_teleop::EvaluateCmdVelSafetyGate(request);

  EXPECT_TRUE(decision.stopped());
  EXPECT_EQ(decision.stop_reason, robot_teleop::CmdVelStopReason::kRuntimeSafetyStop);
  EXPECT_NEAR(decision.output_twist.linear.x, 0.0, 1e-9);
}

TEST(CmdVelSafetyTest, EvaluateCmdVelSafetyGate_UnavailableInput_PublishesZero) {
  robot_teleop::CmdVelSafetyGateRequest request;
  request.input_twist.linear.x = 0.4;
  request.input_available = false;

  const auto decision = robot_teleop::EvaluateCmdVelSafetyGate(request);

  EXPECT_TRUE(decision.stopped());
  EXPECT_EQ(decision.stop_reason, robot_teleop::CmdVelStopReason::kInputUnavailable);
  EXPECT_NEAR(decision.output_twist.linear.x, 0.0, 1e-9);
}

TEST(CmdVelSafetyTest, EvaluateCmdVelSafetyGate_StaleInput_PublishesZero) {
  robot_teleop::CmdVelSafetyGateRequest request;
  request.input_twist.linear.x = 0.4;
  request.input_fresh = false;

  const auto decision = robot_teleop::EvaluateCmdVelSafetyGate(request);

  EXPECT_TRUE(decision.stopped());
  EXPECT_EQ(decision.stop_reason, robot_teleop::CmdVelStopReason::kInputStale);
  EXPECT_NEAR(decision.output_twist.linear.x, 0.0, 1e-9);
}

TEST(CmdVelSafetyTest, EvaluateCmdVelSafetyGate_FreshInput_AppliesDynamicLimit) {
  robot_teleop::CmdVelSafetyGateRequest request;
  request.input_twist.linear.x = 1.0;
  request.input_twist.angular.z = 0.8;
  request.dynamic_speed_limit_mps = 0.5;
  request.dynamic_angular_limit_radps = 0.6;

  const auto decision = robot_teleop::EvaluateCmdVelSafetyGate(request);

  EXPECT_FALSE(decision.stopped());
  EXPECT_EQ(decision.stop_reason, robot_teleop::CmdVelStopReason::kNone);
  EXPECT_NEAR(decision.output_twist.linear.x, 0.5, 1e-9);
  EXPECT_NEAR(decision.output_twist.angular.z, 0.4, 1e-9);
}

TEST(CmdVelSafetyTest, SelectCommandSource_ManualTakeover_ChoosesTeleop) {
  EXPECT_EQ(robot_teleop::SelectCommandSource(true, "nav2"), "teleop");
  EXPECT_EQ(robot_teleop::SelectCommandSource(false, "nav2"), "nav2");
}

TEST(CmdVelSafetyTest, DescribeCommandState_ReportsHighestPriorityState) {
  EXPECT_EQ(robot_teleop::DescribeCommandState(true, true, true, "nav2"), "emergency_stop");
  EXPECT_EQ(robot_teleop::DescribeCommandState(false, true, true, "nav2"), "safety_stop");
  EXPECT_EQ(robot_teleop::DescribeCommandState(false, false, true, "nav2"),
            "manual_takeover:teleop");
  EXPECT_EQ(robot_teleop::DescribeCommandState(false, false, false, "nav2"), "nav2");
}

TEST(CmdVelSafetyTest, SafetyState_ToStringAndPriority_AreStable) {
  EXPECT_EQ(robot_teleop::ToString(robot_teleop::SafetyState::kEmergencyStop),
            "EMERGENCY_STOP");
  EXPECT_GT(robot_teleop::SafetyPriority(robot_teleop::SafetyState::kEmergencyStop),
            robot_teleop::SafetyPriority(robot_teleop::SafetyState::kCommunicationLost));
  EXPECT_GT(robot_teleop::SafetyPriority(robot_teleop::SafetyState::kCommunicationLost),
            robot_teleop::SafetyPriority(robot_teleop::SafetyState::kChassisFault));
  EXPECT_GT(robot_teleop::SafetyPriority(robot_teleop::SafetyState::kChassisFault),
            robot_teleop::SafetyPriority(robot_teleop::SafetyState::kLocalizationLost));
  EXPECT_GT(robot_teleop::SafetyPriority(robot_teleop::SafetyState::kLocalizationLost),
            robot_teleop::SafetyPriority(robot_teleop::SafetyState::kSensorStale));
  EXPECT_GT(robot_teleop::SafetyPriority(robot_teleop::SafetyState::kSensorStale),
            robot_teleop::SafetyPriority(robot_teleop::SafetyState::kManualTakeover));
  EXPECT_GT(robot_teleop::SafetyPriority(robot_teleop::SafetyState::kManualTakeover),
            robot_teleop::SafetyPriority(robot_teleop::SafetyState::kSpeedLimited));
  EXPECT_GT(robot_teleop::SafetyPriority(robot_teleop::SafetyState::kSpeedLimited),
            robot_teleop::SafetyPriority(robot_teleop::SafetyState::kRecovery));
  EXPECT_GT(robot_teleop::SafetyPriority(robot_teleop::SafetyState::kRecovery),
            robot_teleop::SafetyPriority(robot_teleop::SafetyState::kNormal));
}

TEST(CmdVelSafetyTest, ResolveHighestPriority_PicksMostCriticalState) {
  const auto resolved = robot_teleop::ResolveHighestPriority(
      {robot_teleop::SafetyState::kSpeedLimited,
       robot_teleop::SafetyState::kLocalizationLost,
       robot_teleop::SafetyState::kSensorStale});

  EXPECT_EQ(resolved, robot_teleop::SafetyState::kLocalizationLost);
}

TEST(CmdVelSafetyTest, ApplySafetyPolicy_NormalPassesThrough) {
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 0.3;
  twist.angular.z = 0.2;

  const auto output = robot_teleop::ApplySafetyPolicy(
      twist, robot_teleop::SafetyState::kNormal, 0.15, 0.4);

  EXPECT_NEAR(output.linear.x, 0.3, 1e-9);
  EXPECT_NEAR(output.angular.z, 0.2, 1e-9);
}

TEST(CmdVelSafetyTest, ApplySafetyPolicy_SpeedLimitedClampsCommand) {
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 0.5;
  twist.angular.z = 1.2;

  const auto output = robot_teleop::ApplySafetyPolicy(
      twist, robot_teleop::SafetyState::kSpeedLimited, 0.15, 0.4);

  EXPECT_NEAR(output.linear.x, 0.15, 1e-9);
  EXPECT_NEAR(output.angular.z, 0.36, 1e-9);
}

TEST(CmdVelSafetyTest, ApplySafetyPolicy_HighRiskStatesPublishZero) {
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 0.5;
  twist.angular.z = 1.2;

  for (const auto state : {robot_teleop::SafetyState::kSensorStale,
                           robot_teleop::SafetyState::kLocalizationLost,
                           robot_teleop::SafetyState::kChassisFault,
                           robot_teleop::SafetyState::kCommunicationLost,
                           robot_teleop::SafetyState::kEmergencyStop}) {
    const auto output = robot_teleop::ApplySafetyPolicy(twist, state, 0.15, 0.4);
    EXPECT_NEAR(output.linear.x, 0.0, 1e-9);
    EXPECT_NEAR(output.angular.z, 0.0, 1e-9);
  }
}

TEST(CmdVelSafetyTest, SafetyStateFromLocalizationHealth_MapsHealthStrings) {
  EXPECT_EQ(robot_teleop::SafetyStateFromLocalizationHealth("LOCALIZATION_OK localized=true"),
            robot_teleop::SafetyState::kNormal);
  EXPECT_EQ(robot_teleop::SafetyStateFromLocalizationHealth("LOCALIZATION_UNSTABLE covariance"),
            robot_teleop::SafetyState::kSpeedLimited);
  EXPECT_EQ(robot_teleop::SafetyStateFromLocalizationHealth("LOCALIZATION_LOST timeout"),
            robot_teleop::SafetyState::kLocalizationLost);
  EXPECT_EQ(robot_teleop::SafetyStateFromLocalizationHealth("LOCALIZATION_RECOVERING"),
            robot_teleop::SafetyState::kRecovery);
}

TEST(CmdVelSafetyTest, SafetyStateFromChassisStatus_MapsFaultCodes) {
  EXPECT_EQ(robot_teleop::ExtractFaultCodeName("fault_code=NONE(0):estop=0"), "NONE");
  EXPECT_EQ(robot_teleop::SafetyStateFromChassisStatus(true, "fault_code=NONE(0):estop=0"),
            robot_teleop::SafetyState::kNormal);
  EXPECT_EQ(robot_teleop::SafetyStateFromChassisStatus(true, "fault_code=CMD_TIMEOUT(1):estop=0"),
            robot_teleop::SafetyState::kSensorStale);
  EXPECT_EQ(robot_teleop::SafetyStateFromChassisStatus(
                true, "fault_code=HEARTBEAT_TIMEOUT(2):estop=0"),
            robot_teleop::SafetyState::kCommunicationLost);
  EXPECT_EQ(robot_teleop::SafetyStateFromChassisStatus(
                true, "fault_code=BACKEND_DISCONNECTED(3):estop=0"),
            robot_teleop::SafetyState::kCommunicationLost);
  EXPECT_EQ(robot_teleop::SafetyStateFromChassisStatus(
                true, "fault_code=MALFORMED_PACKET(4):estop=0"),
            robot_teleop::SafetyState::kChassisFault);
  EXPECT_EQ(robot_teleop::SafetyStateFromChassisStatus(true, "fault_code=ESTOP_ACTIVE(5):estop=1"),
            robot_teleop::SafetyState::kEmergencyStop);
  EXPECT_EQ(robot_teleop::SafetyStateFromChassisStatus(false, "fault_code=NONE(0):estop=0"),
            robot_teleop::SafetyState::kCommunicationLost);
}

}  // namespace
