#ifndef CYBERGEAR_M5_DRIVER_HH
#define CYBERGEAR_M5_DRIVER_HH

#include <async_comm/serial.h>
#include <cstdint>
#include <type_traits>
#include <unordered_map>
#include <future>
#include <memory>
#include <thread>
#include <string>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <rclcpp/rclcpp.hpp>

#include "common.hh"
#include "cybergear_m5_driver/common.hh"
#include "cybergear_m5_driver/cybergear_m5_packet.hh"


// foward definition
namespace async_comm
{
  class Serial;
}; // end namespace async_comm

namespace cybergear_m5_driver
{

struct MotorStatus
{
  rclcpp::Time stamp;
  uint8_t id;
  float position;
  float velocity;
  float effort;
  float tempareture;
};

class CyberGearM5Driver
{
public:
  typedef std::shared_ptr<CyberGearM5Driver> Ptr;
  typedef std::unordered_map<uint8_t, MotorStatus> MotorStatusMap;

  CyberGearM5Driver();
  virtual ~CyberGearM5Driver();
  bool init(const std::string& port, size_t baudrate, const Uint8Array & motor_ids, ControlMode mode);
  bool shutodwn();
  void enable_motor(uint8_t id, uint8_t mode);
  void disable_motor(uint8_t id);
  void stop_motor(uint8_t id);
  void control_motion(uint8_t id, float position, float velocity, float current);
  void control_position(uint8_t id, float position);
  void control_velocity(uint8_t id, float velocity);
  void control_current(uint8_t id, float current);
  void register_receive_motor_status_callback();

private:
  typedef std::shared_ptr<async_comm::Serial> SerialPtr;
  typedef std::shared_ptr<std::thread> ThreadPtr;

  void pack_send_packet();
  void receive_callback(const uint8_t *buf, size_t len);
  void unpack_receive_packet(const ByteArray& frame);
  void receive_data_process();
  bool get_next_frame(ByteArray & frame);

  // for motor config
  Uint8Array motor_ids_;
  ControlMode mode_;

  // for m5 stack communication
  ByteArray receive_buffer_;
  std::mutex receive_condition_mutex_;            //!< receive condition for serial receive event
  std::mutex receive_buffer_mutex_;               //!< receive buffer mutex
  std::condition_variable receive_process_cond_;  //!< receive process condition for thread sync
  bool is_shutdown_;

  // serial configs
  std::string port_;
  uint16_t baudrate_;
  SerialPtr serial_;

  // serial receive thread
  ThreadPtr receive_thread_;
  MotorStatusMap motor_status_map_;
};

};

#endif // !CYBERGEAR_M5_DRIVER_HH
