#include "robot_hardware/chassis_packet.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace robot_hardware {
namespace {

std::string TrimCarriageReturn(std::string line) {
  if (!line.empty() && line.back() == '\r') {
    line.pop_back();
  }
  return line;
}

void AppendInt16(std::vector<uint8_t>* out, const int16_t value) {
  out->push_back(static_cast<uint8_t>((value >> 8) & 0xff));
  out->push_back(static_cast<uint8_t>(value & 0xff));
}

int16_t ReadInt16(const std::vector<uint8_t>& payload, const std::size_t offset) {
  return static_cast<int16_t>(
      static_cast<uint16_t>(payload.at(offset) << 8) | static_cast<uint16_t>(payload.at(offset + 1)));
}

uint16_t ReadUint16(const std::vector<uint8_t>& payload, const std::size_t offset) {
  return static_cast<uint16_t>(
      static_cast<uint16_t>(payload.at(offset) << 8) | static_cast<uint16_t>(payload.at(offset + 1)));
}

int16_t ClampToInt16(const double value) {
  const double clamped = std::clamp(value, -32768.0, 32767.0);
  return static_cast<int16_t>(std::lround(clamped));
}

bool ParseBoolToken(const std::string& token, bool* value) {
  if (token == "1" || token == "true" || token == "TRUE") {
    *value = true;
    return true;
  }
  if (token == "0" || token == "false" || token == "FALSE") {
    *value = false;
    return true;
  }
  return false;
}

}  // namespace

std::string FaultCodeName(const ChassisFaultCode code) {
  switch (code) {
    case ChassisFaultCode::kNone:
      return "NONE";
    case ChassisFaultCode::kCommandTimeout:
      return "CMD_TIMEOUT";
    case ChassisFaultCode::kHeartbeatTimeout:
      return "HEARTBEAT_TIMEOUT";
    case ChassisFaultCode::kBackendDisconnected:
      return "BACKEND_DISCONNECTED";
    case ChassisFaultCode::kMalformedPacket:
      return "MALFORMED_PACKET";
    case ChassisFaultCode::kEmergencyStopActive:
      return "ESTOP_ACTIVE";
  }
  return "UNKNOWN";
}

int FaultCodeValue(const ChassisFaultCode code) {
  return static_cast<int>(code);
}

std::optional<ChassisFaultCode> FaultCodeFromValue(const int value) {
  switch (value) {
    case 0:
      return ChassisFaultCode::kNone;
    case 1:
      return ChassisFaultCode::kCommandTimeout;
    case 2:
      return ChassisFaultCode::kHeartbeatTimeout;
    case 3:
      return ChassisFaultCode::kBackendDisconnected;
    case 4:
      return ChassisFaultCode::kMalformedPacket;
    case 5:
      return ChassisFaultCode::kEmergencyStopActive;
    default:
      return std::nullopt;
  }
}

std::string EncodeCommand(const ChassisCommand& command) {
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out << std::setprecision(4) << "CMD " << command.linear_x_mps << " "
      << command.linear_y_mps << " "
      << command.angular_z_radps << "\n";
  return out.str();
}

std::string EncodeCommandV2(const ChassisCommandFrameV2& frame) {
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out << std::setprecision(6) << "CMDV2 " << frame.seq << " " << frame.timestamp_sec << " "
      << frame.mode << " " << frame.command.linear_x_mps << " " << frame.command.linear_y_mps
      << " " << frame.command.angular_z_radps << " " << frame.max_linear_velocity_mps << " "
      << frame.max_angular_velocity_radps << " " << (frame.emergency_stop ? 1 : 0) << "\n";
  return out.str();
}

std::string EncodeOdometry(const ChassisOdometryPacket& odometry) {
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out << std::setprecision(4) << "ODOM " << odometry.x_m << " " << odometry.y_m << " "
      << odometry.yaw_rad << " " << odometry.linear_x_mps << " " << odometry.angular_z_radps;
  if (odometry.battery_voltage.has_value()) {
    out << " " << *odometry.battery_voltage;
  }
  out << "\n";
  return out.str();
}

std::string EncodeOdometryV2(const ChassisOdometryPacket& odometry) {
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out << std::setprecision(6) << "ODOMV2 " << odometry.seq.value_or(0) << " "
      << odometry.timestamp_sec.value_or(0.0) << " " << odometry.x_m << " " << odometry.y_m
      << " " << odometry.yaw_rad << " " << odometry.linear_x_mps << " "
      << odometry.linear_y_mps << " " << odometry.angular_z_radps << " "
      << odometry.left_rpm.value_or(0.0) << " " << odometry.right_rpm.value_or(0.0) << "\n";
  return out.str();
}

std::string EncodeState(const ChassisStatePacket& state) {
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out << "STATE " << state.status;
  if (state.battery_voltage.has_value()) {
    out << " " << std::setprecision(4) << *state.battery_voltage;
  }
  out << "\n";
  return out.str();
}

std::string EncodeStateV2(const ChassisStatePacket& state) {
  std::ostringstream out;
  out.setf(std::ios::fixed);
  const std::string mode =
      !state.mode.empty() ? state.mode : (state.status.empty() ? "unknown" : state.status);
  out << std::setprecision(6) << "STATEV2 " << state.seq.value_or(0) << " "
      << state.timestamp_sec.value_or(0.0) << " " << mode << " "
      << state.battery_voltage.value_or(0.0) << " " << FaultCodeValue(state.fault_code) << " "
      << (state.emergency_stop ? 1 : 0) << " " << (state.connected ? 1 : 0) << " "
      << state.temperature_c.value_or(0.0) << "\n";
  return out.str();
}

std::string EncodeHeartbeatV2(const ChassisHeartbeatPacket& heartbeat) {
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out << std::setprecision(6) << "HEARTBEATV2 " << heartbeat.seq << " "
      << heartbeat.timestamp_sec << " " << heartbeat.source << "\n";
  return out.str();
}

std::optional<ChassisCommand> ParseCommandLine(const std::string& raw_line) {
  const std::string line = TrimCarriageReturn(raw_line);
  std::istringstream in(line);
  std::string tag;
  ChassisCommand command;
  if (!(in >> tag >> command.linear_x_mps) || tag != "CMD") {
    return std::nullopt;
  }
  double second = 0.0;
  if (!(in >> second)) {
    return std::nullopt;
  }
  double third = 0.0;
  if (in >> third) {
    command.linear_y_mps = second;
    command.angular_z_radps = third;
  } else {
    command.angular_z_radps = second;
  }
  std::string trailing;
  if (in >> trailing) {
    return std::nullopt;
  }
  return command;
}

std::optional<ChassisCommandFrameV2> ParseCommandFrameV2(const std::string& raw_line) {
  const std::string line = TrimCarriageReturn(raw_line);
  std::istringstream in(line);
  std::string tag;
  ChassisCommandFrameV2 frame;
  int estop = 0;
  if (!(in >> tag >> frame.seq >> frame.timestamp_sec >> frame.mode >>
        frame.command.linear_x_mps >> frame.command.linear_y_mps >>
        frame.command.angular_z_radps >> frame.max_linear_velocity_mps >>
        frame.max_angular_velocity_radps >> estop) ||
      tag != "CMDV2") {
    return std::nullopt;
  }
  frame.emergency_stop = estop != 0;
  std::string trailing;
  if (in >> trailing) {
    return std::nullopt;
  }
  return frame;
}

std::optional<ChassisHeartbeatPacket> ParseHeartbeatFrameV2(const std::string& raw_line) {
  const std::string line = TrimCarriageReturn(raw_line);
  std::istringstream in(line);
  std::string tag;
  ChassisHeartbeatPacket heartbeat;
  if (!(in >> tag >> heartbeat.seq >> heartbeat.timestamp_sec >> heartbeat.source) ||
      tag != "HEARTBEATV2") {
    return std::nullopt;
  }
  std::string trailing;
  if (in >> trailing) {
    return std::nullopt;
  }
  return heartbeat;
}

std::optional<ChassisPacket> ParsePacketLine(const std::string& raw_line) {
  const std::string line = TrimCarriageReturn(raw_line);
  std::istringstream in(line);
  std::string tag;
  in >> tag;
  if (tag == "ODOM") {
    ChassisOdometryPacket odometry;
    if (!(in >> odometry.x_m >> odometry.y_m >> odometry.yaw_rad >> odometry.linear_x_mps >>
          odometry.angular_z_radps)) {
      return std::nullopt;
    }
    double battery = 0.0;
    if (in >> battery) {
      odometry.battery_voltage = battery;
    }
    std::string trailing;
    if (in >> trailing) {
      return std::nullopt;
    }
    ChassisPacket packet;
    packet.kind = ChassisPacketKind::kOdometry;
    packet.odometry = odometry;
    return packet;
  }

  if (tag == "ODOMV2") {
    ChassisOdometryPacket odometry;
    uint64_t seq = 0;
    double timestamp_sec = 0.0;
    double left_rpm = 0.0;
    double right_rpm = 0.0;
    if (!(in >> seq >> timestamp_sec >> odometry.x_m >> odometry.y_m >> odometry.yaw_rad >>
          odometry.linear_x_mps >> odometry.linear_y_mps >> odometry.angular_z_radps >>
          left_rpm >> right_rpm)) {
      return std::nullopt;
    }
    odometry.seq = seq;
    odometry.timestamp_sec = timestamp_sec;
    odometry.left_rpm = left_rpm;
    odometry.right_rpm = right_rpm;
    std::string trailing;
    if (in >> trailing) {
      return std::nullopt;
    }
    ChassisPacket packet;
    packet.kind = ChassisPacketKind::kOdometry;
    packet.odometry = odometry;
    return packet;
  }

  if (tag == "STATE") {
    ChassisStatePacket state;
    if (!(in >> state.status)) {
      return std::nullopt;
    }
    double battery = 0.0;
    if (in >> battery) {
      state.battery_voltage = battery;
    }
    std::string trailing;
    if (in >> trailing) {
      return std::nullopt;
    }
    ChassisPacket packet;
    packet.kind = ChassisPacketKind::kState;
    packet.state = state;
    return packet;
  }

  if (tag == "STATEV2") {
    ChassisStatePacket state;
    uint64_t seq = 0;
    double timestamp_sec = 0.0;
    double battery = 0.0;
    int fault_code = 0;
    std::string estop_token;
    std::string connected_token;
    double temperature = 0.0;
    if (!(in >> seq >> timestamp_sec >> state.mode >> battery >> fault_code >>
          estop_token >> connected_token >> temperature)) {
      return std::nullopt;
    }
    bool estop = false;
    bool connected = false;
    if (!ParseBoolToken(estop_token, &estop) || !ParseBoolToken(connected_token, &connected)) {
      return std::nullopt;
    }
    const auto mapped_fault = FaultCodeFromValue(fault_code);
    if (!mapped_fault.has_value()) {
      return std::nullopt;
    }
    state.seq = seq;
    state.timestamp_sec = timestamp_sec;
    state.status = state.mode;
    state.battery_voltage = battery;
    state.fault_code = *mapped_fault;
    state.emergency_stop = estop;
    state.connected = connected;
    state.temperature_c = temperature;
    std::string trailing;
    if (in >> trailing) {
      return std::nullopt;
    }
    ChassisPacket packet;
    packet.kind = ChassisPacketKind::kState;
    packet.state = state;
    return packet;
  }

  if (tag == "HEARTBEATV2") {
    if (auto heartbeat = ParseHeartbeatFrameV2(line)) {
      ChassisPacket packet;
      packet.kind = ChassisPacketKind::kHeartbeat;
      packet.heartbeat = *heartbeat;
      return packet;
    }
    return std::nullopt;
  }

  return std::nullopt;
}

std::vector<uint8_t> EncodeMickBinaryFrame(
    const uint8_t type, const std::vector<uint8_t>& payload) {
  std::vector<uint8_t> frame;
  frame.reserve(payload.size() + 7);
  frame.push_back(0xae);
  frame.push_back(0xea);
  frame.push_back(static_cast<uint8_t>(payload.size() + 2U));
  frame.push_back(type);
  frame.insert(frame.end(), payload.begin(), payload.end());
  uint8_t checksum = 0;
  for (std::size_t i = 2; i < frame.size(); ++i) {
    checksum = static_cast<uint8_t>(checksum + frame[i]);
  }
  frame.push_back(checksum);
  frame.push_back(0xef);
  frame.push_back(0xfe);
  return frame;
}

std::vector<uint8_t> EncodeMickSpeedCommand(const ChassisCommand& command) {
  std::vector<uint8_t> payload;
  payload.reserve(6);
  AppendInt16(&payload, ClampToInt16(command.linear_x_mps * 1000.0));
  AppendInt16(&payload, ClampToInt16(command.linear_y_mps * 1000.0));
  AppendInt16(&payload, ClampToInt16(command.angular_z_radps * 1000.0));
  return EncodeMickBinaryFrame(0xf3, payload);
}

std::vector<uint8_t> EncodeMickClearOdometryCommand() {
  return EncodeMickBinaryFrame(0xe1, {0x01, 0x00, 0x00, 0x00});
}

std::optional<ChassisPacket> DecodeMickBinaryPacket(const MickBinaryFrame& frame) {
  if (frame.type == 0xa1 && frame.payload.size() >= 12U) {
    ChassisOdometryPacket odometry;
    odometry.x_m = ReadInt16(frame.payload, 0) / 1000.0;
    odometry.y_m = ReadInt16(frame.payload, 2) / 1000.0;
    odometry.yaw_rad = ReadInt16(frame.payload, 4) / 1000.0;
    odometry.linear_x_mps = ReadInt16(frame.payload, 6) / 1000.0;
    odometry.angular_z_radps = ReadInt16(frame.payload, 8) / 1000.0;
    odometry.battery_voltage = ReadUint16(frame.payload, 10) / 100.0;
    ChassisPacket packet;
    packet.kind = ChassisPacketKind::kOdometry;
    packet.odometry = odometry;
    return packet;
  }

  if (frame.type == 0xa2) {
    ChassisStatePacket state;
    state.status = frame.payload.empty() || frame.payload[0] == 0 ? "running" : "fault";
    if (frame.payload.size() >= 3U) {
      state.battery_voltage = ReadUint16(frame.payload, 1) / 100.0;
    }
    ChassisPacket packet;
    packet.kind = ChassisPacketKind::kState;
    packet.state = state;
    return packet;
  }

  return std::nullopt;
}

ChassisPacketStream::ChassisPacketStream(const std::size_t max_buffer_bytes)
    : max_buffer_bytes_(max_buffer_bytes) {}

void ChassisPacketStream::Append(const char* data, const std::size_t size) {
  buffer_.append(data, size);
  if (buffer_.size() > max_buffer_bytes_) {
    buffer_.clear();
    overflowed_ = true;
  }
}

void ChassisPacketStream::Append(const std::string& data) { Append(data.data(), data.size()); }

std::vector<ChassisPacket> ChassisPacketStream::DrainPackets() {
  std::vector<ChassisPacket> packets;
  std::size_t newline = std::string::npos;
  while ((newline = buffer_.find('\n')) != std::string::npos) {
    const std::string line = buffer_.substr(0, newline);
    buffer_.erase(0, newline + 1);
    if (auto packet = ParsePacketLine(line)) {
      packets.push_back(*packet);
    } else if (!line.empty()) {
      ++invalid_line_count_;
    }
  }
  return packets;
}

bool ChassisPacketStream::Overflowed() const { return overflowed_; }

std::size_t ChassisPacketStream::InvalidLineCount() const { return invalid_line_count_; }

void ChassisPacketStream::ClearOverflow() { overflowed_ = false; }

void ChassisPacketStream::ClearInvalidLineCount() { invalid_line_count_ = 0; }

void ChassisPacketStream::Clear() {
  buffer_.clear();
  overflowed_ = false;
  invalid_line_count_ = 0;
}

MickBinaryPacketStream::MickBinaryPacketStream(const std::size_t max_buffer_bytes)
    : max_buffer_bytes_(max_buffer_bytes) {}

void MickBinaryPacketStream::Append(const char* data, const std::size_t size) {
  buffer_.insert(buffer_.end(), data, data + size);
  if (buffer_.size() > max_buffer_bytes_) {
    buffer_.clear();
    overflowed_ = true;
  }
}

void MickBinaryPacketStream::Append(const std::string& data) {
  Append(data.data(), data.size());
}

std::vector<ChassisPacket> MickBinaryPacketStream::DrainPackets() {
  std::vector<ChassisPacket> packets;
  while (buffer_.size() >= 7U) {
    const std::array<uint8_t, 2> magic{0xae, 0xea};
    const auto header = std::search(buffer_.begin(), buffer_.end(), magic.begin(), magic.end());
    if (header == buffer_.end()) {
      buffer_.clear();
      return packets;
    }
    buffer_.erase(buffer_.begin(), header);
    if (buffer_.size() < 7U) {
      return packets;
    }
    const std::size_t length = buffer_[2];
    const std::size_t total_length = 2U + 1U + length + 2U;
    if (length < 2U) {
      buffer_.erase(buffer_.begin());
      ++invalid_frame_count_;
      continue;
    }
    if (buffer_.size() < total_length) {
      return packets;
    }
    if (buffer_[total_length - 2U] != 0xef || buffer_[total_length - 1U] != 0xfe) {
      buffer_.erase(buffer_.begin());
      ++invalid_frame_count_;
      continue;
    }
    uint8_t checksum = 0;
    for (std::size_t i = 2; i < total_length - 3U; ++i) {
      checksum = static_cast<uint8_t>(checksum + buffer_[i]);
    }
    if (checksum != buffer_[total_length - 3U]) {
      buffer_.erase(buffer_.begin(), buffer_.begin() + total_length);
      ++invalid_frame_count_;
      continue;
    }
    MickBinaryFrame frame;
    frame.type = buffer_[3];
    frame.payload.assign(buffer_.begin() + 4, buffer_.begin() + total_length - 3U);
    if (auto packet = DecodeMickBinaryPacket(frame)) {
      packets.push_back(*packet);
    }
    buffer_.erase(buffer_.begin(), buffer_.begin() + total_length);
  }
  return packets;
}

bool MickBinaryPacketStream::Overflowed() const { return overflowed_; }

std::size_t MickBinaryPacketStream::InvalidFrameCount() const { return invalid_frame_count_; }

void MickBinaryPacketStream::ClearOverflow() { overflowed_ = false; }

void MickBinaryPacketStream::ClearInvalidFrameCount() { invalid_frame_count_ = 0; }

void MickBinaryPacketStream::Clear() {
  buffer_.clear();
  overflowed_ = false;
  invalid_frame_count_ = 0;
}

}  // namespace robot_hardware
