#ifndef CYBER_GEAR_M5_PAKCET_HH
#define CYBER_GEAR_M5_PAKCET_HH

#include "cybergear_m5_driver/common.hh"
#include <sys/types.h>
#include <vector>
#include <cstdint>

namespace cybergear_m5_driver
{

enum class ControlMode
{
  POSITION,
  VELOCITY,
  CURRENT,
  MOTION
};

/**
 * @brief Packet class
 */
class Packet
{
public:
  static constexpr uint8_t Header = 0x89;
  static constexpr uint8_t CommandPacketSize = 0x08;

  enum class CommandPacketIndex
  {
    FrameHeader = 0,
    PacketType,
    TargetMotorId,
    DataFrameSize,
    PacketSequence,
    RequsetType,
    Optional,
    CheckSum,
    PacketSize
  };

  explicit Packet(uint8_t type, uint8_t id, uint16_t size) : type_(type), id_(id), size_(size){}
  virtual ~Packet() {}
  uint8_t type() const { return type_; }
  uint8_t id() const { return id_; }

private:
  uint8_t type_;
  uint8_t id_;
  uint8_t size_;
};


/**
 * @brief Response packet to PC
 */
class RequestPacket : public Packet
{
public:
  enum class Type
  {
    Enable = 0,
    Reset,
    GetMotorIdList,
    GetControlMode,
    ControlMotion,
    ControlSpeed,
    ControlPosition,
    ControlCurrent,
    _INVALID_TYPE_RANGE
  };

  RequestPacket(uint8_t type, uint8_t id, uint16_t size, uint8_t seq);
  virtual ~RequestPacket();
  virtual bool pack();
  const ByteArray & packet();
  const ByteArray &command_packet() const;
  const ByteArray &data_packet() const;

protected:
  uint8_t calc_checksum(const ByteArray& packet) const;
  uint8_t packet_sequence_;
  ByteArray command_packet_;
  ByteArray data_packet_;
  ByteArray packet_;
};


/**
 * @brief Request packet from PC
 */
class ResponsePacket : public Packet
{
public:
  enum class Type
  {
    MotorStatus = 0,
    _INVALID_TYPE_RANGE
  };

  ResponsePacket(uint8_t type, uint16_t size, const ByteArray &packet);

  virtual bool unpack();
  bool validate() const;
  uint8_t command_packet_id() const {return packet_id_; };
  uint8_t command_packet_type() const {return packet_type_; };
  uint8_t command_packet_sequence() const {return packet_sequence_; };
  uint8_t command_packet_optional1() const {return packet_optional1_; };
  uint8_t command_packet_optional2() const {return packet_optional2_; };

protected:
  bool check_checksum() const;
  const ByteArray &command_packet() const {return command_packet_; }
  const ByteArray &data_packet() const {return data_packet_; }
  bool check_packet_type() const { return type() == command_packet_[static_cast<uint8_t>(Packet::CommandPacketIndex::PacketType)]; }

private:
  ByteArray command_packet_;
  ByteArray data_packet_;
  uint8_t packet_header_;
  uint8_t packet_id_;
  uint8_t packet_type_;
  uint8_t packet_data_size_;
  uint8_t packet_sequence_;
  uint8_t packet_optional1_;
  uint8_t packet_optional2_;
  uint8_t packet_checksum_;
};


/**
 * @brief EnableRequestPacket class
 */
class EnableRequestPacket : public RequestPacket
{
public:
  static const int PacketSize = CommandPacketSize;
  EnableRequestPacket(uint8_t id, uint8_t control_mode, uint8_t seq);
  virtual ~EnableRequestPacket();
  virtual bool pack();

private:
  uint8_t control_mode_;
};


/**
 * @brief ControlPositionRequestPacket class
 */
class ControlPositionRequestPacket : public RequestPacket
{
public:
  static const int PacketSize = CommandPacketSize + 4 + 1;
  explicit ControlPositionRequestPacket(uint8_t id, float position, uint8_t seq);
  virtual ~ControlPositionRequestPacket();
  virtual bool pack();
  float ref_position() const { return ref_position_; }

private:
  float ref_position_;
};


class ControlSpeedRequestPacket : public RequestPacket
{
public:
  static const int PacketSize = CommandPacketSize + 4 + 1;
  explicit ControlSpeedRequestPacket(uint8_t id, float speed, uint8_t seq);
  virtual ~ControlSpeedRequestPacket();
  virtual bool pack();
  float ref_speed() const { return ref_speed_; }

private:
  float ref_speed_;
};


class ControlCurrentRequestPacket : public RequestPacket
{
public:
  static const int PacketSize = CommandPacketSize + 4 + 1;
  explicit ControlCurrentRequestPacket(uint8_t id, float current, uint8_t seq);
  virtual ~ControlCurrentRequestPacket();
  virtual bool pack();
  float ref_current() const { return ref_current_; }

private:
  float ref_current_;
};


class ControlMotionRequestPacket : public RequestPacket
{
public:
  static const int PacketSize = CommandPacketSize + 12 + 1;
  explicit ControlMotionRequestPacket(uint8_t id, float position, float speed, float current, uint8_t seq);
  virtual ~ControlMotionRequestPacket();
  virtual bool pack();
  float ref_position() const { return ref_position_; }
  float ref_speed() const { return ref_speed_; }
  float ref_current() const { return ref_current_; }

private:
  float ref_position_;
  float ref_speed_;
  float ref_current_;
};


class MotorStatusResponsePacket : public ResponsePacket
{
public:
  static const int PacketSize = CommandPacketSize + 16 + 1;
  explicit MotorStatusResponsePacket(const ByteArray& packet);
  virtual ~MotorStatusResponsePacket();
  virtual bool unpack();
  float position() const { return position_; }
  float velocity() const { return position_; }
  float effort() const { return effort_; }
  float tempareture() const { return tempareture_; }

private:
  float position_;
  float velocity_;
  float effort_;
  float tempareture_;
};

}; // end namespace cybergear_m5_driver

#endif // CYBER_GEAR_BRIDGE_PACKET_HH
