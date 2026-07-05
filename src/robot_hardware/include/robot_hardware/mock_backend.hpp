#pragma once

#include "robot_hardware/chassis_backend.hpp"

namespace robot_hardware {

class MockBackend final : public ChassisBackend {
 public:
  MockBackend() = default;

  std::string Name() const override;
  bool Open(std::string* error) override;
  void Close() override;
  bool IsOpen() const override;
  bool WriteCommand(const ChassisCommand& command, std::string* error) override;
  bool WriteCommandV2(const ChassisCommandFrameV2& frame, std::string* error) override;
  bool WriteHeartbeatV2(const ChassisHeartbeatPacket& heartbeat, std::string* error) override;
  std::optional<std::string> Read(std::string* error) override;
  ChassisCommand LastCommand() const;
  std::optional<ChassisCommandFrameV2> LastCommandFrameV2() const;
  std::optional<ChassisHeartbeatPacket> LastHeartbeatV2() const;

 private:
  bool open_ = false;
  ChassisCommand last_command_;
  std::optional<ChassisCommandFrameV2> last_command_frame_v2_;
  std::optional<ChassisHeartbeatPacket> last_heartbeat_v2_;
};

}  // namespace robot_hardware
