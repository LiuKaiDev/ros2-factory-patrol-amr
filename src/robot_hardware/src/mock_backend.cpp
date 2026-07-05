#include "robot_hardware/mock_backend.hpp"

namespace robot_hardware {

std::string MockBackend::Name() const { return "mock"; }

bool MockBackend::Open(std::string*) {
  open_ = true;
  return true;
}

void MockBackend::Close() { open_ = false; }

bool MockBackend::IsOpen() const { return open_; }

bool MockBackend::WriteCommand(const ChassisCommand& command, std::string*) {
  last_command_ = command;
  return open_;
}

bool MockBackend::WriteCommandV2(const ChassisCommandFrameV2& frame, std::string*) {
  last_command_frame_v2_ = frame;
  last_command_ = frame.command;
  return open_;
}

bool MockBackend::WriteHeartbeatV2(const ChassisHeartbeatPacket& heartbeat, std::string*) {
  last_heartbeat_v2_ = heartbeat;
  return open_;
}

std::optional<std::string> MockBackend::Read(std::string*) { return std::nullopt; }

ChassisCommand MockBackend::LastCommand() const { return last_command_; }

std::optional<ChassisCommandFrameV2> MockBackend::LastCommandFrameV2() const {
  return last_command_frame_v2_;
}

std::optional<ChassisHeartbeatPacket> MockBackend::LastHeartbeatV2() const {
  return last_heartbeat_v2_;
}

}  // namespace robot_hardware
