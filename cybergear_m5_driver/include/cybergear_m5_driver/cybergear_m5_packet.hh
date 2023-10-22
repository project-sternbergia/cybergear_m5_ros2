#ifndef CYBER_GEAR_M5_PAKCET_HH
#define CYBER_GEAR_M5_PAKCET_HH

#include "cybergear_m5_driver/common.hh"
#include "cybergear_m5_driver/cybergear_driver_defs.hh"
#include <sys/types.h>
#include <vector>
#include <cstdint>

namespace cybergear_m5_driver
{

enum class ControlMode
{
  Position = MODE_POSITION,
  Velocity = MODE_SPEED,
  Current = MODE_CURRENT,
  Motion = MODE_MOTION
};

/**
 * @brief Packet class
 */
class Packet
{
public:
  static constexpr uint8_t Header = 0x89;               //!< Requeset / Response packet header
  static constexpr uint8_t CommandPacketSize = 0x08;    //!< Command packet size

  /**
   * @brief Command packet index
   */
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

  /**
   * @brief Construct a new Packet object
   *
   * @param type  packet type
   * @param id    motor id
   * @param size  packet size
   */
  explicit Packet(uint8_t type, uint8_t id, uint16_t size) : type_(type), id_(id), size_(size){}
  virtual ~Packet() {}

  // accessor
  uint8_t type() const { return type_; }
  uint8_t id() const { return id_; }

private:
  uint8_t type_;      //!< packet type
  uint8_t id_;        //!< motor can id
  uint8_t size_;      //!< packet size
};


/**
 * @brief Response packet to PC
 */
class RequestPacket : public Packet
{
public:
  enum class Type
  {
    Enable = 0,           //!< enable request packet type
    Reset,                //!< reset request packet type
    // GetMotorIdList,    //!< get motor id list packet type (not implemented)
    // GetControlMode,    //!< get control mode packet type (not implemented)
    ControlMotion,        //!< control motion request packet type
    ControlSpeed,         //!< control speed request packet type
    ControlPosition,      //!< control position request packet type
    ControlCurrent,       //!< control current request packet type
    SetMechPosToZero,     //!< set current mechanical encoder position to zero
    // GetMotorParameter,
    SetLimitSpeed,        //!< set limit speed
    SetLimitCurrent,      //!< set limit current
    SetLimitTorque,       //!< set limit torque
    // SetCurrentParameter,
    _INVALID_TYPE_RANGE
  };

  /**
   * @brief Construct a new Request Packet object
   *
   * @param type  packet type
   * @param id    motor can id
   * @param size  packet size
   * @param seq   sequence count (not use. for checking packet drop)
   */
  RequestPacket(uint8_t type, uint8_t id, uint16_t size, uint8_t seq);
  virtual ~RequestPacket();

  /**
   * @brief Pack request pacekt
   *
   * @return true   success
   * @return false  failed
   */
  virtual bool pack();

  // accessors
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
 * @brief ResetRequestPacket class
 */
class ResetRequestPacket : public RequestPacket
{
public:
  static const int PacketSize = CommandPacketSize;
  ResetRequestPacket(uint8_t id, uint8_t seq);
  virtual ~ResetRequestPacket();
};


/**
 * @brief SetMechPosToZeroRequestPacket class
*/
class SetMechPosToZeroRequestPacket : public RequestPacket
{
public:
  static const int PacketSize = CommandPacketSize;
  SetMechPosToZeroRequestPacket(uint8_t id, uint8_t seq);
  virtual ~SetMechPosToZeroRequestPacket() {}
};


/**
 * @brief SetLimitSpeedRequestPacket class
*/
class SetLimitSpeedRequestPacket : public RequestPacket
{
public:
  static const int PacketSize = CommandPacketSize + 4 + 1;
  SetLimitSpeedRequestPacket(uint8_t id, float speed, uint8_t seq);
  virtual ~SetLimitSpeedRequestPacket() {}
  virtual bool pack();

private:
  float limit_speed_;
};


/**
 * @brief SetLimitCrrentRequestPacket class
*/
class SetLimitCurrentRequestPacket : public RequestPacket
{
public:
  static const int PacketSize = CommandPacketSize + 4 + 1;
  SetLimitCurrentRequestPacket(uint8_t id, float current, uint8_t seq);
  virtual ~SetLimitCurrentRequestPacket() {}
  virtual bool pack();

private:
  float limit_current_;
};


/**
 * @brief SetLimitTorqueRequestPacket class
*/
class SetLimitTorqueRequestPacket : public RequestPacket
{
public:
  static const int PacketSize = CommandPacketSize + 4 + 1;
  SetLimitTorqueRequestPacket(uint8_t id, float torque, uint8_t seq);
  virtual ~SetLimitTorqueRequestPacket() {}
  virtual bool pack();

private:
  float limit_torque_;
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


/**
 * @brief ControlSpeedRequestePacket class
 */
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


/**
 * @brief ControlCurrentRequestPacket class
 */
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


/**
 * @brief ControlMotionRequestPacket class
 */
class ControlMotionRequestPacket : public RequestPacket
{
public:
  static const int PacketSize = CommandPacketSize + 20 + 1;
  explicit ControlMotionRequestPacket(uint8_t id, float position, float speed, float current, float kp, float kd, uint8_t seq);
  virtual ~ControlMotionRequestPacket();
  virtual bool pack();
  float ref_position() const { return ref_position_; }
  float ref_speed() const { return ref_speed_; }
  float ref_current() const { return ref_current_; }
  float kp() const { return kp_; }
  float kd() const { return kd_; }

private:
  float ref_position_;
  float ref_speed_;
  float ref_current_;
  float kp_;
  float kd_;
};


/**
 * @brief MotorStatusResponsePacket class
 */
class MotorStatusResponsePacket : public ResponsePacket
{
public:
  static const int PacketSize = CommandPacketSize + 16 + 1;
  explicit MotorStatusResponsePacket(const ByteArray& packet);
  virtual ~MotorStatusResponsePacket();
  virtual bool unpack();
  float position() const { return position_; }
  float velocity() const { return velocity_; }
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
