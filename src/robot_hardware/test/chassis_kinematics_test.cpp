#include "robot_hardware/chassis_kinematics.hpp"

#include <cmath>

#include <gtest/gtest.h>

namespace robot_hardware {
namespace {

constexpr double kWheelDiameterM = 0.15;
constexpr double kWheelBaseM = 0.42;
constexpr double kTrackWidthM = 0.43;

}  // namespace

TEST(ChassisKinematicsMathTest, DiffDriveStraightLineProducesEqualWheelSpeeds) {
  ChassisCommand command;
  command.linear_x_mps = 0.4;

  const auto speeds =
      EstimateWheelSpeeds(command, "diff_drive", kWheelDiameterM, kWheelBaseM, kTrackWidthM);

  EXPECT_NEAR(speeds.rpm[0], speeds.rpm[1], 1e-9);
  EXPECT_NEAR(speeds.rpm[2], speeds.rpm[3], 1e-9);
  EXPECT_GT(speeds.rpm[0], 0.0);
}

TEST(ChassisKinematicsMathTest, DiffDriveInPlaceRotationProducesOppositeWheelSpeeds) {
  ChassisCommand command;
  command.angular_z_radps = 0.8;

  const auto speeds =
      EstimateWheelSpeeds(command, "diff_drive", kWheelDiameterM, kWheelBaseM, kTrackWidthM);

  EXPECT_NEAR(speeds.rpm[0], -speeds.rpm[1], 1e-9);
  EXPECT_NEAR(speeds.rpm[2], -speeds.rpm[3], 1e-9);
  EXPECT_LT(speeds.rpm[0], 0.0);
  EXPECT_GT(speeds.rpm[1], 0.0);
}

TEST(ChassisKinematicsMathTest, WheelDiameterChangesRpmForSameLinearCommand) {
  ChassisCommand command;
  command.linear_x_mps = 0.4;

  const auto small_wheel =
      EstimateWheelSpeeds(command, "diff_drive", 0.10, kWheelBaseM, kTrackWidthM);
  const auto large_wheel =
      EstimateWheelSpeeds(command, "diff_drive", 0.20, kWheelBaseM, kTrackWidthM);

  EXPECT_GT(std::abs(small_wheel.rpm[0]), std::abs(large_wheel.rpm[0]));
}

TEST(ChassisKinematicsMathTest, TrackWidthChangesRotationWheelSpeed) {
  ChassisCommand command;
  command.angular_z_radps = 0.8;

  const auto narrow =
      EstimateWheelSpeeds(command, "diff_drive", kWheelDiameterM, kWheelBaseM, 0.30);
  const auto wide =
      EstimateWheelSpeeds(command, "diff_drive", kWheelDiameterM, kWheelBaseM, 0.60);

  EXPECT_GT(std::abs(wide.rpm[0]), std::abs(narrow.rpm[0]));
}

TEST(ChassisKinematicsMathTest, WheelSpeedsRoundTripPreservesCommandDirection) {
  ChassisCommand command;
  command.linear_x_mps = 0.35;
  command.angular_z_radps = -0.4;

  const auto speeds =
      EstimateWheelSpeeds(command, "diff_drive", kWheelDiameterM, kWheelBaseM, kTrackWidthM);
  const auto estimated =
      EstimateCommandFromWheelSpeeds(speeds, "diff_drive", kWheelDiameterM, kWheelBaseM, kTrackWidthM);

  EXPECT_NEAR(estimated.linear_x_mps, command.linear_x_mps, 1e-9);
  EXPECT_NEAR(estimated.angular_z_radps, command.angular_z_radps, 1e-9);
  EXPECT_LT(estimated.angular_z_radps, 0.0);
}

TEST(ChassisKinematicsMathTest, InvalidGeometryInputsDoNotCrash) {
  ChassisCommand command;
  command.linear_x_mps = 0.3;
  command.angular_z_radps = 0.2;

  const auto speeds = EstimateWheelSpeeds(command, "diff_drive", 0.0, kWheelBaseM, 0.0);
  const auto estimated =
      EstimateCommandFromWheelSpeeds(speeds, "diff_drive", 0.0, kWheelBaseM, 0.0);

  EXPECT_DOUBLE_EQ(speeds.rpm[0], 0.0);
  EXPECT_DOUBLE_EQ(speeds.rpm[1], 0.0);
  EXPECT_DOUBLE_EQ(estimated.linear_x_mps, 0.0);
  EXPECT_DOUBLE_EQ(estimated.angular_z_radps, 0.0);
}

}  // namespace robot_hardware
