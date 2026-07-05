#pragma once

#include <cstddef>
#include <netinet/in.h>

#include "robot_hardware/chassis_backend.hpp"

namespace robot_hardware {

class UdpBackend final : public ChassisBackend {
 public:
  explicit UdpBackend(ChassisBackendConfig config);
  ~UdpBackend() override;

  std::string Name() const override;
  bool Open(std::string* error) override;
  void Close() override;
  bool IsOpen() const override;
  bool WriteCommand(const ChassisCommand& command, std::string* error) override;
  bool WriteCommandV2(const ChassisCommandFrameV2& frame, std::string* error) override;
  bool WriteHeartbeatV2(const ChassisHeartbeatPacket& heartbeat, std::string* error) override;
  std::optional<std::string> Read(std::string* error) override;

 private:
  bool SendBytes(const char* data, std::size_t size, std::string* error);
  ChassisBackendConfig config_;
  int fd_ = -1;
  sockaddr_in remote_addr_ {};
};

}  // namespace robot_hardware
