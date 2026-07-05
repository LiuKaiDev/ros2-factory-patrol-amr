#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace robot_hardware {

struct ChassisCommand {
  double linear_x_mps = 0.0;
  double linear_y_mps = 0.0;
  double angular_z_radps = 0.0;
};

enum class ChassisFaultCode {
  kNone = 0,
  kCommandTimeout = 1,
  kHeartbeatTimeout = 2,
  kBackendDisconnected = 3,
  kMalformedPacket = 4,
  kEmergencyStopActive = 5,
};

std::string FaultCodeName(ChassisFaultCode code);
int FaultCodeValue(ChassisFaultCode code);
std::optional<ChassisFaultCode> FaultCodeFromValue(int value);

struct ChassisCommandFrameV2 {
  uint64_t seq = 0;
  double timestamp_sec = 0.0;
  std::string mode = "velocity";
  ChassisCommand command;
  double max_linear_velocity_mps = 0.0;
  double max_angular_velocity_radps = 0.0;
  bool emergency_stop = false;
};

struct ChassisOdometryPacket {
  std::optional<uint64_t> seq;
  std::optional<double> timestamp_sec;
  double x_m = 0.0;
  double y_m = 0.0;
  double yaw_rad = 0.0;
  double linear_x_mps = 0.0;
  double linear_y_mps = 0.0;
  double angular_z_radps = 0.0;
  std::optional<double> left_rpm;
  std::optional<double> right_rpm;
  std::optional<double> battery_voltage;
};

struct ChassisStatePacket {
  std::optional<uint64_t> seq;
  std::optional<double> timestamp_sec;
  std::string status;
  std::string mode;
  std::optional<double> battery_voltage;
  ChassisFaultCode fault_code = ChassisFaultCode::kNone;
  bool emergency_stop = false;
  bool connected = true;
  std::optional<double> temperature_c;
};

struct ChassisHeartbeatPacket {
  uint64_t seq = 0;
  double timestamp_sec = 0.0;
  std::string source;
};

enum class ChassisPacketKind { kOdometry, kState, kHeartbeat };

struct ChassisPacket {
  ChassisPacketKind kind = ChassisPacketKind::kState;
  ChassisOdometryPacket odometry;
  ChassisStatePacket state;
  ChassisHeartbeatPacket heartbeat;
};

std::string EncodeCommand(const ChassisCommand& command);
std::string EncodeCommandV2(const ChassisCommandFrameV2& frame);
std::string EncodeOdometry(const ChassisOdometryPacket& odometry);
std::string EncodeOdometryV2(const ChassisOdometryPacket& odometry);
std::string EncodeState(const ChassisStatePacket& state);
std::string EncodeStateV2(const ChassisStatePacket& state);
std::string EncodeHeartbeatV2(const ChassisHeartbeatPacket& heartbeat);
std::optional<ChassisCommand> ParseCommandLine(const std::string& line);
std::optional<ChassisCommandFrameV2> ParseCommandFrameV2(const std::string& line);
std::optional<ChassisPacket> ParsePacketLine(const std::string& line);
std::optional<ChassisHeartbeatPacket> ParseHeartbeatFrameV2(const std::string& line);

struct MickBinaryFrame {
  uint8_t type = 0;
  std::vector<uint8_t> payload;
};

std::vector<uint8_t> EncodeMickBinaryFrame(uint8_t type, const std::vector<uint8_t>& payload);
std::vector<uint8_t> EncodeMickSpeedCommand(const ChassisCommand& command);
std::vector<uint8_t> EncodeMickClearOdometryCommand();
std::optional<ChassisPacket> DecodeMickBinaryPacket(const MickBinaryFrame& frame);

class ChassisPacketStream {
 public:
  explicit ChassisPacketStream(std::size_t max_buffer_bytes = 4096);

  void Append(const char* data, std::size_t size);
  void Append(const std::string& data);
  std::vector<ChassisPacket> DrainPackets();
  bool Overflowed() const;
  std::size_t InvalidLineCount() const;
  void ClearOverflow();
  void ClearInvalidLineCount();
  void Clear();

 private:
  std::size_t max_buffer_bytes_;
  std::string buffer_;
  bool overflowed_ = false;
  std::size_t invalid_line_count_ = 0;
};

class MickBinaryPacketStream {
 public:
  explicit MickBinaryPacketStream(std::size_t max_buffer_bytes = 4096);

  void Append(const char* data, std::size_t size);
  void Append(const std::string& data);
  std::vector<ChassisPacket> DrainPackets();
  bool Overflowed() const;
  std::size_t InvalidFrameCount() const;
  void ClearOverflow();
  void ClearInvalidFrameCount();
  void Clear();

 private:
  std::size_t max_buffer_bytes_;
  std::vector<uint8_t> buffer_;
  bool overflowed_ = false;
  std::size_t invalid_frame_count_ = 0;
};

}  // namespace robot_hardware
