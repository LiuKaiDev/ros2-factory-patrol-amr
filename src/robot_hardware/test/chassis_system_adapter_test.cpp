#include "robot_hardware/chassis_system_adapter.hpp"

#include <deque>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include <gtest/gtest.h>

#include "robot_hardware/mock_backend.hpp"

namespace robot_hardware {
namespace {

class ScriptedBackend final : public ChassisBackend {
 public:
  explicit ScriptedBackend(std::deque<std::string> reads) : reads_(std::move(reads)) {}

  std::string Name() const override { return "scripted"; }

  bool Open(std::string*) override {
    open_ = true;
    return true;
  }

  void Close() override { open_ = false; }

  bool IsOpen() const override { return open_; }

  bool WriteCommand(const ChassisCommand& command, std::string*) override {
    last_command_ = command;
    return open_;
  }

  std::optional<std::string> Read(std::string*) override {
    if (reads_.empty()) {
      return std::nullopt;
    }
    std::string next = reads_.front();
    reads_.pop_front();
    return next;
  }

  ChassisCommand LastCommand() const { return last_command_; }

 private:
  bool open_ = false;
  std::deque<std::string> reads_;
  ChassisCommand last_command_;
};

}  // namespace

TEST(ChassisSystemAdapterTest, Constructor_NullBackend_ThrowsInvalidArgument) {
  EXPECT_THROW(ChassisSystemAdapter(nullptr), std::invalid_argument);
}

TEST(ChassisSystemAdapterTest, Write_ClosedBackend_ReturnsError) {
  ChassisSystemAdapter adapter(std::make_unique<MockBackend>());

  std::string error;
  const bool ok = adapter.Write(ChassisCommand{}, &error);

  EXPECT_FALSE(ok);
  EXPECT_EQ(error, "chassis backend is not open");
}

TEST(ChassisSystemAdapterTest, Read_MockDiffDriveCommand_IntegratesPlanarState) {
  auto backend = std::make_unique<MockBackend>();
  const MockBackend* backend_ptr = backend.get();
  ChassisSystemAdapter adapter(std::move(backend));
  ASSERT_TRUE(adapter.Open(nullptr));
  ChassisCommand command;
  command.linear_x_mps = 0.5;
  command.linear_y_mps = 0.25;
  command.angular_z_radps = 0.2;
  ASSERT_TRUE(adapter.Write(command, nullptr));

  ChassisSystemState state;
  ASSERT_TRUE(adapter.Read(2.0, &state, nullptr));

  EXPECT_TRUE(state.backend_open);
  EXPECT_TRUE(state.connected);
  EXPECT_EQ(state.backend_name, "mock");
  EXPECT_NEAR(state.x_m, 1.0, 1e-9);
  EXPECT_NEAR(state.y_m, 0.0, 1e-9);
  EXPECT_NEAR(state.yaw_rad, 0.4, 1e-9);
  EXPECT_NEAR(state.linear_x_mps, 0.5, 1e-9);
  EXPECT_NEAR(state.linear_y_mps, 0.0, 1e-9);
  EXPECT_NEAR(state.angular_z_radps, 0.2, 1e-9);
  EXPECT_NEAR(backend_ptr->LastCommand().linear_y_mps, 0.0, 1e-9);
  EXPECT_NE(state.wheel_speeds.rpm[0], state.wheel_speeds.rpm[1]);
}

TEST(ChassisSystemAdapterTest, Read_MockMecanumCommand_PreservesSidewaysVelocity) {
  auto backend = std::make_unique<MockBackend>();
  const MockBackend* backend_ptr = backend.get();
  ChassisSystemAdapterConfig config;
  config.kinematics_model = "mecanum";
  ChassisSystemAdapter adapter(std::move(backend), config);
  ASSERT_TRUE(adapter.Open(nullptr));
  ChassisCommand command;
  command.linear_x_mps = 0.5;
  command.linear_y_mps = 0.25;
  ASSERT_TRUE(adapter.Write(command, nullptr));

  ChassisSystemState state;
  ASSERT_TRUE(adapter.Read(1.0, &state, nullptr));

  EXPECT_NEAR(state.x_m, 0.5, 1e-9);
  EXPECT_NEAR(state.y_m, 0.25, 1e-9);
  EXPECT_NEAR(state.linear_y_mps, 0.25, 1e-9);
  EXPECT_NEAR(backend_ptr->LastCommand().linear_y_mps, 0.25, 1e-9);
}

TEST(ChassisSystemAdapterTest, Write_CommandFrameV2_NormalizesAndStoresMetadata) {
  auto backend = std::make_unique<MockBackend>();
  const MockBackend* backend_ptr = backend.get();
  ChassisSystemAdapter adapter(std::move(backend));
  ASSERT_TRUE(adapter.Open(nullptr));
  ChassisCommandFrameV2 frame;
  frame.seq = 5;
  frame.timestamp_sec = 12.0;
  frame.mode = "velocity";
  frame.command.linear_x_mps = 0.5;
  frame.command.linear_y_mps = 0.25;
  frame.command.angular_z_radps = 0.1;

  ASSERT_TRUE(adapter.Write(frame, nullptr));

  const auto stored = backend_ptr->LastCommandFrameV2();
  ASSERT_TRUE(stored.has_value());
  EXPECT_EQ(stored->seq, 5U);
  EXPECT_NEAR(stored->command.linear_x_mps, 0.5, 1e-9);
  EXPECT_NEAR(stored->command.linear_y_mps, 0.0, 1e-9);
  EXPECT_NEAR(stored->command.angular_z_radps, 0.1, 1e-9);
}

TEST(ChassisSystemAdapterTest, Read_MockFourWheelSteerAliasDegradesToDiffDrive) {
  auto backend = std::make_unique<MockBackend>();
  const MockBackend* backend_ptr = backend.get();
  ChassisSystemAdapterConfig config;
  config.kinematics_model = "four_ws4wd";
  ChassisSystemAdapter adapter(std::move(backend), config);
  ASSERT_TRUE(adapter.Open(nullptr));
  ChassisCommand command;
  command.linear_x_mps = 0.5;
  command.linear_y_mps = 0.25;
  ASSERT_TRUE(adapter.Write(command, nullptr));

  ChassisSystemState state;
  ASSERT_TRUE(adapter.Read(1.0, &state, nullptr));

  EXPECT_NEAR(state.x_m, 0.5, 1e-9);
  EXPECT_NEAR(state.y_m, 0.0, 1e-9);
  EXPECT_NEAR(state.linear_y_mps, 0.0, 1e-9);
  EXPECT_NEAR(backend_ptr->LastCommand().linear_y_mps, 0.0, 1e-9);
}

TEST(ChassisSystemAdapterTest, Read_TextPackets_UpdatesMeasuredState) {
  auto backend = std::make_unique<ScriptedBackend>(
      std::deque<std::string>{"ODOM 1.25 0.10 0.02 0.30 -0.04 24.1\nSTATE simulated 23.8\n"});
  ChassisSystemAdapter adapter(std::move(backend));
  ASSERT_TRUE(adapter.Open(nullptr));

  ChassisSystemState state;
  ASSERT_TRUE(adapter.Read(0.02, &state, nullptr));

  EXPECT_TRUE(state.connected);
  EXPECT_NEAR(state.x_m, 1.25, 1e-9);
  EXPECT_NEAR(state.y_m, 0.10, 1e-9);
  EXPECT_NEAR(state.yaw_rad, 0.02, 1e-9);
  EXPECT_NEAR(state.linear_x_mps, 0.30, 1e-9);
  EXPECT_NEAR(state.angular_z_radps, -0.04, 1e-9);
  EXPECT_EQ(state.hardware_status, "simulated");
  EXPECT_NEAR(state.battery_voltage, 23.8, 1e-9);
  EXPECT_EQ(state.packet_sequence, 2U);
}

TEST(ChassisSystemAdapterTest, Read_TextV2Packets_UpdatesHeartbeatAndFaultMetadata) {
  auto backend = std::make_unique<ScriptedBackend>(
      std::deque<std::string>{"HEARTBEATV2 7 10.0 chassis\nSTATEV2 8 10.1 velocity 23.8 5 1 0 36.5\n"});
  ChassisSystemAdapter adapter(std::move(backend));
  ASSERT_TRUE(adapter.Open(nullptr));

  ChassisSystemState state;
  EXPECT_FALSE(adapter.Read(0.02, &state, nullptr));

  EXPECT_FALSE(state.connected);
  EXPECT_EQ(state.last_rx_seq, 8U);
  EXPECT_DOUBLE_EQ(state.last_rx_timestamp_sec, 10.1);
  EXPECT_EQ(state.hardware_status, "velocity");
  EXPECT_EQ(state.fault_code, ChassisFaultCode::kEmergencyStopActive);
  EXPECT_TRUE(state.emergency_stop);
  EXPECT_DOUBLE_EQ(state.temperature_c, 36.5);
  EXPECT_EQ(state.packet_sequence, 2U);
}

TEST(ChassisSystemAdapterTest, SetKinematicsModel_DiffDriveToMecanum_PreservesSidewaysVelocity) {
  auto backend = std::make_unique<MockBackend>();
  const MockBackend* backend_ptr = backend.get();
  ChassisSystemAdapter adapter(std::move(backend));
  ASSERT_TRUE(adapter.Open(nullptr));
  adapter.SetKinematicsModel("mecanum");
  ChassisCommand command;
  command.linear_y_mps = 0.25;
  ASSERT_TRUE(adapter.Write(command, nullptr));

  ChassisSystemState state;
  ASSERT_TRUE(adapter.Read(1.0, &state, nullptr));

  EXPECT_NEAR(state.y_m, 0.25, 1e-9);
  EXPECT_NEAR(backend_ptr->LastCommand().linear_y_mps, 0.25, 1e-9);
}

TEST(ChassisSystemAdapterTest, ResetOdometry_IntegratedMockState_ClearsPoseOnly) {
  ChassisSystemAdapter adapter(std::make_unique<MockBackend>());
  ASSERT_TRUE(adapter.Open(nullptr));
  ChassisCommand command;
  command.linear_x_mps = 0.5;
  ASSERT_TRUE(adapter.Write(command, nullptr));
  ChassisSystemState state;
  ASSERT_TRUE(adapter.Read(1.0, &state, nullptr));
  ASSERT_NEAR(state.x_m, 0.5, 1e-9);

  adapter.ResetOdometry();
  state = adapter.State();

  EXPECT_NEAR(state.x_m, 0.0, 1e-9);
  EXPECT_NEAR(state.y_m, 0.0, 1e-9);
  EXPECT_NEAR(state.yaw_rad, 0.0, 1e-9);
  EXPECT_NEAR(state.linear_x_mps, 0.5, 1e-9);
}

TEST(ChassisSystemAdapterTest, Read_MalformedTextPacket_ReturnsObservableError) {
  auto backend = std::make_unique<ScriptedBackend>(std::deque<std::string>{"NOPE bad packet\n"});
  ChassisSystemAdapter adapter(std::move(backend));
  ASSERT_TRUE(adapter.Open(nullptr));

  std::string error;
  ChassisSystemState state;
  const bool ok = adapter.Read(0.02, &state, &error);

  EXPECT_FALSE(ok);
  EXPECT_EQ(error, "ignored malformed chassis packet line");
  EXPECT_FALSE(state.connected);
  EXPECT_EQ(state.fault_code, ChassisFaultCode::kMalformedPacket);
}

TEST(ChassisSystemAdapterTest, Close_OpenBackend_DropsConnectionState) {
  ChassisSystemAdapter adapter(std::make_unique<MockBackend>());
  ASSERT_TRUE(adapter.Open(nullptr));

  adapter.Close();

  EXPECT_FALSE(adapter.State().backend_open);
  EXPECT_FALSE(adapter.State().connected);
}

}  // namespace robot_hardware
