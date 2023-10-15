#include "cybergear_m5_driver/cybergear_m5_driver.hh"
#include <endian.h>
#include <cstdint>
#include <memory>
#include <mutex>


using namespace cybergear_m5_driver;

CyberGearM5Driver::CyberGearM5Driver()
  : receive_buffer_()
  , motor_ids_()
  , mode_(cybergear_m5_driver::ControlMode::POSITION)
  , receive_condition_mutex_()
  , receive_buffer_mutex_()
  , receive_process_cond_()
  , is_shutdown_(false)
{}

CyberGearM5Driver::~CyberGearM5Driver()
{}

bool CyberGearM5Driver::init(const std::string& port, size_t baudrate, const Uint8Array & motor_ids, ControlMode mode)
{
  // open seiral port
  {
    serial_ = std::make_shared<async_comm::Serial>(port, baudrate);
    serial_->register_receive_callback(std::bind(&CyberGearM5Driver::receive_callback, this, std::placeholders::_1, std::placeholders::_2));
    if (!serial_->init()) {
      std::cerr << "failed to init serial port" << std::endl;
    }
  }

  // create receive thread
  receive_thread_ = std::make_shared<std::thread>(std::bind(&CyberGearM5Driver::receive_data_process, this));

  return true;
}

bool CyberGearM5Driver::shutodwn()
{
  is_shutdown_ = true;
  receive_process_cond_.notify_one();

  // wait fin thread
  if (receive_thread_) {
    receive_thread_->join();
  }

  // close serial
  if (serial_) {
    serial_->close();
  }
  return true;
}

void CyberGearM5Driver::enable_motor(uint8_t id, uint8_t mode)
{}

void CyberGearM5Driver::disable_motor(uint8_t id)
{}

void CyberGearM5Driver::stop_motor(uint8_t id)
{}

void CyberGearM5Driver::control_motion(uint8_t id, float position, float velocity, float current)
{
 ControlMotionRequestPacket request(id, position, velocity, current, 0);
  if (request.pack()) {
    const ByteArray & packet = request.packet();
    serial_->send_bytes(packet.data(), packet.size());
  }
}

void CyberGearM5Driver::control_position(uint8_t id, float position)
{
  ControlPositionRequestPacket request(id, position, 0);
  if (request.pack()) {
    const ByteArray & packet = request.packet();
    serial_->send_bytes(packet.data(), packet.size());
  }
}

void CyberGearM5Driver::control_velocity(uint8_t id, float velocity)
{
  ControlSpeedRequestPacket request(id, velocity, 0);
  if (request.pack()) {
    const ByteArray & packet = request.packet();
    serial_->send_bytes(packet.data(), packet.size());
  }
}

void CyberGearM5Driver::control_current(uint8_t id, float current)
{
  ControlCurrentRequestPacket request(id, current, 0);
  if (request.pack()) {
    const ByteArray & packet = request.packet();
    serial_->send_bytes(packet.data(), packet.size());
  }
}

void CyberGearM5Driver::receive_callback(const uint8_t *buf, size_t len)
{
  {
    std::lock_guard<std::mutex> lock(receive_buffer_mutex_);
    ByteArray receive_data(buf, buf + len);
    receive_buffer_.insert(receive_buffer_.end(), receive_data.begin(), receive_data.end());
  }
  receive_process_cond_.notify_one();
}

void CyberGearM5Driver::unpack_receive_packet(const ByteArray& frame)
{
  ResponsePacket::Type packet_type = ResponsePacket::Type(frame[static_cast<uint8_t>(Packet::CommandPacketIndex::PacketType)]);
  if (packet_type == ResponsePacket::Type::MotorStatus) {
    auto status = MotorStatusResponsePacket(frame);
    if (status.unpack()) {
      MotorStatus mot;
      mot.stamp = rclcpp::Clock().now();
      mot.position = status.position();
      mot.velocity = status.velocity();
      mot.effort = status.effort();
      mot.tempareture = status.tempareture();
      motor_status_map_[status.id()] = mot;
    }
  }
}

void CyberGearM5Driver::receive_data_process()
{
  while (!is_shutdown_) {
    std::unique_lock<std::mutex> lock(receive_condition_mutex_);
    receive_process_cond_.wait(lock);

    // process next frame
    ByteArray frame;
    if (get_next_frame(frame)) {
      unpack_receive_packet(frame);
    }
  }
}

bool CyberGearM5Driver::get_next_frame(ByteArray & frame)
{
  // clear return value
  frame.clear();

  // lock
  std::lock_guard<std::mutex> lock(receive_buffer_mutex_);

  // check data size
  if (receive_buffer_.size() < Packet::CommandPacketSize) {
    return false;
  }

  // find header
  auto header_itr = std::find(receive_buffer_.begin(), receive_buffer_.end(), Packet::Header);
  if (header_itr == receive_buffer_.end()) {
    receive_buffer_.clear();
    return false;
  }

  receive_buffer_.erase(receive_buffer_.begin(), header_itr);

  // check data size
  size_t data_frame_size = receive_buffer_[static_cast<uint8_t>(Packet::CommandPacketIndex::DataFrameSize)];
  size_t frame_size = Packet::CommandPacketSize + data_frame_size;

  // check cmd frame and data frame size
  frame = ByteArray(receive_buffer_.begin(), receive_buffer_.begin() + frame_size);
  receive_buffer_.erase(receive_buffer_.begin(), receive_buffer_.begin() + frame_size);

  return true;
}

