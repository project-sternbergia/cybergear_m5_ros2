#ifndef CYBERGEAR_M5_DRIVER_HH
#define CYBERGEAR_M5_DRIVER_HH

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

struct MotorCommand
{
  rclcpp::Time stamp;
  uint8_t id;
  float position;
  float velocity;
  float effort;
};

class CyberGearM5Driver
{
public:
  typedef std::shared_ptr<CyberGearM5Driver> Ptr;
  typedef std::unordered_map<uint8_t, MotorStatus> MotorStatusMap;

  /**
   * @brief Construct a new Cyber Gear M 5 Driver object
   */
  CyberGearM5Driver();

  /**
   * @brief Destroy the Cyber Gear M 5 Driver object
   */
  virtual ~CyberGearM5Driver();

  /**
   * @brief Init m5 stack connection
   *
   * @param port      serial port
   * @param baudrate  baudrate
   * @return true     success to init
   * @return false    failed to init
   */
  bool init(const std::string& port, size_t baudrate);

  /**
   * @brief Shutdown m5 stack connection
   *
   * @return true   OK
   * @return false  NG
   */
  bool shutdown();

  /**
   * @brief Send enable motor command
   *
   * @param id    target motor id
   * @param mode  target control mode (position, velocity, effort or motion)
   */
  void enable_motor(uint8_t id, ControlMode mode);

  /**
   * @brief Send reset motor commmand
   *
   * @param id target motor id
   */
  void reset_motor(uint8_t id);

  /**
   * @brief Set mechanical position to zero command
   *
   * @param id target motor id
   */
  void set_mech_position_to_zero(uint8_t id);

  /**
   * @brief Set the limit speed
   *
   * @param id      target motor id
   * @param speed   limit speed [rad/sec]
   */
  void set_limit_speed(uint8_t id, float speed);

  /**
   * @brief Set the limit current
   *
   * @param id      target motor id
   * @param current limit current [A]
   */
  void set_limit_current(uint8_t id, float current);

  /**
   * @brief Set the limit torque
   *
   * @param id      target motor id
   * @param torque  limit torque [Nm]
   */
  void set_limit_torque(uint8_t id, float torque);

  /**
   * @brief Set the control parameter. This control gain uses at pos control mode.
   *
   * @param id  motor can id
   * @param kp  default value is 30 range = [0:200]
   */
  void set_position_control_gain(uint8_t id, float kp);

  /**
   * @brief Set the velocity parametr. This control gain uses at pos and vel control mode.
   *
   * @param id  motor can id
   * @param kp  default value is 2 range = [0:200]
   * @param kd  default value is 0.021 range = [0:200]
   */
  void set_velocity_control_gain(uint8_t id, float kp, float kd);

  /**
   * @brief Control motor motion
   *
   * @param id        motor id
   * @param position  target position [rad]
   * @param velocity  target velocity [rad/s]
   * @param current   target current [A]
   */
  void control_motion(uint8_t id, float position, float velocity, float current, float kp = 100.0f, float kd = 10.0f);

  /**
   * @brief Control motor position
   *
   * @param id        motor id
   * @param position  target position [rad]
   */
  void control_position(uint8_t id, float position);

  /**
   * @brief Control motor velocity
   *
   * @param id        motor id
   * @param velocity  target motor velocity [rad/s]
   */
  void control_velocity(uint8_t id, float velocity);

  /**
   * @brief Control motor current
   *
   * @param id      motor id
   * @param current target motor current [A]
   */
  void control_current(uint8_t id, float current);

  /**
   * @brief Get the motor status
   *
   * @param id      motor id
   * @param status  current motor status
   * @return true   success
   * @return false  failed
   */
  bool get_motor_status(uint8_t id, MotorStatus & status) const;

private:
  typedef std::shared_ptr<async_comm::Serial> SerialPtr;
  typedef std::shared_ptr<std::thread> ThreadPtr;

  /**
   * @brief Receive callback for async io
   *
   * @param buf receive data
   * @param len receive data length
   */
  void receive_callback(const uint8_t *buf, size_t len);

  /**
   * @brief Unpack receive packet
   *
   * @param frame received frame
   */
  void unpack_receive_packet(const ByteArray& frame);

  /**
   * @brief Process received data
   */
  void receive_data_process();

  /**
   * @brief Get next frame
   *
   * @param frame received frame
   * @return true success
   * @return false failed
   */
  bool get_next_frame(ByteArray & frame);

  // for motor config
  ControlMode mode_;

  // for m5 stack communication
  ByteArray receive_buffer_;
  std::mutex receive_condition_mutex_;            //!< receive condition for serial receive event
  std::mutex receive_buffer_mutex_;               //!< receive buffer mutex
  std::condition_variable receive_process_cond_;  //!< receive process condition for thread sync
  bool is_shutdown_;                              //!< shutdown flag
  uint32_t sequence_count_;                       //!< for sequence count byte

  // serial configs
  std::string port_;    //!< serial port
  uint16_t baudrate_;   //!< serial port baudrate
  SerialPtr serial_;    //!< serial port control object

  // serial receive thread
  ThreadPtr receive_thread_;        //!< receive thread
  MotorStatusMap motor_status_map_; //!< motor status map
};

};

#endif // !CYBERGEAR_M5_DRIVER_HH
