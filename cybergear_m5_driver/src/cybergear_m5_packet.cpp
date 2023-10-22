#include "cybergear_m5_driver/cybergear_m5_packet.hh"
#include <memory.h>
#include <iostream>

namespace cybergear_m5_driver {

RequestPacket::RequestPacket(uint8_t type, uint8_t id, uint16_t size, uint8_t seq)
  : Packet(type, id, size)
  , packet_sequence_(seq)
  , command_packet_(CommandPacketSize)
  , data_packet_(size - CommandPacketSize)
  , packet_(size)
{}

RequestPacket::~RequestPacket()
{}

bool RequestPacket::pack()
{
  if (packet_.size() < CommandPacketSize) {
    return false;
  }

  // pack command packet
  command_packet_[0] = Packet::Header;
  command_packet_[1] = static_cast<uint8_t>(type());
  command_packet_[2] = id();
  command_packet_[3] = packet_.size() - CommandPacketSize;
  command_packet_[4] = packet_sequence_;
  command_packet_[5] = 0x00;
  command_packet_[6] = 0x00;
  command_packet_[7] = calc_checksum(command_packet_);
  return true;
}

const ByteArray & RequestPacket::packet()
{
  // copy command and data to packet
  std::copy(command_packet_.begin(), command_packet_.end(), packet_.begin());
  std::copy(data_packet_.begin(), data_packet_.end(), packet_.begin() + command_packet_.size());
  return packet_;
}

uint8_t RequestPacket::calc_checksum(const ByteArray& packet) const
{
  uint8_t checksum = 0x00;
  for (uint8_t idx = 0; idx < packet.size() - 1; ++idx) {
    checksum = (checksum + packet[idx]) & 0xFF;
  }
  return checksum;
}


ResponsePacket::ResponsePacket(uint8_t type, uint16_t size, const ByteArray &packet)
    : Packet(type, packet[2], size)
{
  command_packet_ = ByteArray(packet.begin(), packet.begin() + CommandPacketSize);
  data_packet_ = ByteArray(packet.begin() + command_packet_.size(), packet.end());
}

bool ResponsePacket::unpack()
{
  // check packet size
  if (command_packet_.size() != CommandPacketSize) {
    return false;
  }

  packet_header_ = command_packet_[0];
  packet_type_ = command_packet_[1];
  packet_id_ = command_packet_[2];
  packet_data_size_ = command_packet_[3];
  packet_sequence_ = command_packet_[4];
  packet_optional1_ = command_packet_[5];
  packet_optional2_ = command_packet_[6];
  packet_checksum_ = command_packet_[7];
  return validate();
}

bool ResponsePacket::check_checksum() const
{
  // check command packet
  {
    uint8_t check_sum = 0x00;
    for (uint8_t idx = 0; idx < command_packet_.size() - 1; ++idx) {
      check_sum = (check_sum + command_packet_[idx]) & 0xFF;
    }
    if (check_sum != command_packet_.back()) return false;
  }

  // check data packet
  if (data_packet_.size() > 0) {
    uint8_t check_sum = 0x00;
    for (uint8_t idx = 0; idx < data_packet_.size() - 1; ++idx) {
      check_sum = (check_sum + data_packet_[idx]) & 0xFF;
    }
    if (check_sum != data_packet_.back()) return false;
  }
  return true;
}

bool ResponsePacket::validate() const
{
  // check header
  if (packet_header_ != Packet::Header) {
    std::cerr << "header error" << std::endl;
    return false;
  }
  if (packet_type_ >= static_cast<uint8_t>(Type::_INVALID_TYPE_RANGE)) {
    std::cerr << "invalid packet type" << std::endl;
    return false;
  }
  if (packet_data_size_ != data_packet_.size()) {
    std::cerr << "invalid data packet size" << std::endl;
    return false;
  }

  return check_checksum();
}

const ByteArray &RequestPacket::command_packet() const
{
  return command_packet_;
}

const ByteArray &RequestPacket::data_packet() const
{
  return data_packet_;
}


EnableRequestPacket::EnableRequestPacket(uint8_t id, uint8_t control_mode, uint8_t seq)
  : RequestPacket(static_cast<uint8_t>(Type::Enable), id, PacketSize, seq)
  , control_mode_(control_mode)
{}

EnableRequestPacket::~EnableRequestPacket()
{}

bool EnableRequestPacket::pack()
{
  if (!RequestPacket::pack()) return false;
  command_packet_[5] = control_mode_; // use optional-1
  command_packet_[7] = calc_checksum(command_packet_);
  return true;
}


ResetRequestPacket::ResetRequestPacket(uint8_t id, uint8_t seq)
  : RequestPacket(static_cast<uint8_t>(Type::Reset), id, PacketSize, seq)
{}

ResetRequestPacket::~ResetRequestPacket()
{}

// SetMechPosToZeroRequestPacket class
SetMechPosToZeroRequestPacket::SetMechPosToZeroRequestPacket(uint8_t id, uint8_t seq)
  : RequestPacket(static_cast<uint8_t>(Type::Reset), id, PacketSize, seq)
{}

// SetLimitSpeedRequestPacket class
SetLimitSpeedRequestPacket::SetLimitSpeedRequestPacket(uint8_t id, float speed, uint8_t seq)
  : RequestPacket(static_cast<uint8_t>(Type::SetLimitSpeed), id, PacketSize, seq)
  , limit_speed_(std::max(std::min(speed, V_MAX), 0.0f))
{}

bool SetLimitSpeedRequestPacket::pack()
{
  if (!RequestPacket::pack()) return false;
  memcpy(data_packet_.data(), &limit_speed_, sizeof(float));
  data_packet_[4] = calc_checksum(data_packet_);
  return true;
}

// SetLimitCurrentRequestPacket class
SetLimitCurrentRequestPacket::SetLimitCurrentRequestPacket(uint8_t id, float current, uint8_t seq)
  : RequestPacket(static_cast<uint8_t>(Type::SetLimitCurrent), id, PacketSize, seq)
  , limit_current_(std::max(std::min(current, IQ_MAX), 0.0f))
{}

bool SetLimitCurrentRequestPacket::pack()
{
  if (!RequestPacket::pack()) return false;
  memcpy(data_packet_.data(), &limit_current_, sizeof(float));
  data_packet_[4] = calc_checksum(data_packet_);
  return true;
}

// SetLimitTorqueRequestPacket class
SetLimitTorqueRequestPacket::SetLimitTorqueRequestPacket(uint8_t id, float torque, uint8_t seq)
  : RequestPacket(static_cast<uint8_t>(Type::SetLimitTorque), id, PacketSize, seq)
  , limit_torque_(std::max(std::min(torque, T_MAX), 0.0f))
{}

bool SetLimitTorqueRequestPacket::pack()
{
  if (!RequestPacket::pack()) return false;
  memcpy(data_packet_.data(), &limit_torque_, sizeof(float));
  data_packet_[4] = calc_checksum(data_packet_);
  return true;
}

// ControlPositionRequestPacket class
ControlPositionRequestPacket::ControlPositionRequestPacket(uint8_t id, float position, uint8_t seq)
  : RequestPacket(static_cast<uint8_t>(Type::ControlPosition), id, PacketSize, seq)
  , ref_position_(position)
{}

ControlPositionRequestPacket::~ControlPositionRequestPacket()
{}

bool ControlPositionRequestPacket::pack()
{
  if (!RequestPacket::pack()) return false;
  memcpy(data_packet_.data(), &ref_position_, sizeof(float));
  data_packet_[4] = calc_checksum(data_packet_);
  return true;
}


// ControlSpeedRequestPacket class
ControlSpeedRequestPacket::ControlSpeedRequestPacket(uint8_t id, float speed, uint8_t seq)
  : RequestPacket(static_cast<uint8_t>(Type::ControlSpeed), id, PacketSize, seq)
  , ref_speed_(speed)
{}

ControlSpeedRequestPacket::~ControlSpeedRequestPacket()
{}

bool ControlSpeedRequestPacket::pack()
{
  if (!RequestPacket::pack()) return false;
  memcpy(data_packet_.data(), &ref_speed_, sizeof(float));
  data_packet_[4] = calc_checksum(data_packet_);
  return true;
}


// ControlCurrentRequestPacket class
ControlCurrentRequestPacket::ControlCurrentRequestPacket(uint8_t id, float current, uint8_t seq)
  : RequestPacket(static_cast<uint8_t>(Type::ControlCurrent), id, PacketSize, seq)
  , ref_current_(current)
{}

ControlCurrentRequestPacket::~ControlCurrentRequestPacket()
{}

bool ControlCurrentRequestPacket::pack()
{
  if (!RequestPacket::pack()) return false;
  memcpy(data_packet_.data(), &ref_current_, sizeof(float));
  data_packet_[4] = calc_checksum(data_packet_);
  return true;
}


// ControlCurrentRequestPacket class
ControlMotionRequestPacket::ControlMotionRequestPacket(uint8_t id, float position, float speed, float current, float kp, float kd, uint8_t seq)
  : RequestPacket(static_cast<uint8_t>(Type::ControlCurrent), id, PacketSize, seq)
  , ref_position_(position)
  , ref_speed_(speed)
  , ref_current_(current)
  , kp_(kp)
  , kd_(kd)
{}

ControlMotionRequestPacket::~ControlMotionRequestPacket()
{}

bool ControlMotionRequestPacket::pack()
{
  if (!RequestPacket::pack()) return false;
  memcpy(data_packet_.data() + 0, &ref_position_, sizeof(float));
  memcpy(data_packet_.data() + 4, &ref_speed_, sizeof(float));
  memcpy(data_packet_.data() + 8, &ref_current_, sizeof(float));
  memcpy(data_packet_.data() + 12, &kp_, sizeof(float));
  memcpy(data_packet_.data() + 16, &kd_, sizeof(float));
  data_packet_[20] = calc_checksum(data_packet_);
  return true;
}


// Response Packet
MotorStatusResponsePacket::MotorStatusResponsePacket(const ByteArray & packet)
  : ResponsePacket(static_cast<uint8_t>(ResponsePacket::Type::MotorStatus), PacketSize, packet)
  , position_(0.0)
  , velocity_(0.0)
  , effort_(0.0)
  , tempareture_(0.0)
{}

MotorStatusResponsePacket::~MotorStatusResponsePacket()
{}

bool MotorStatusResponsePacket::unpack()
{
  if (!ResponsePacket::unpack()) return false;

  memcpy(&position_, data_packet().data() + 0, sizeof(position_));
  memcpy(&velocity_, data_packet().data() + 4, sizeof(velocity_));
  memcpy(&effort_, data_packet().data() + 8, sizeof(effort_));
  memcpy(&tempareture_, data_packet().data() + 12, sizeof(tempareture_));
  return true;
}

}
