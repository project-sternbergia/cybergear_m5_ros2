#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "cybergear_m5_driver/cybergear_m5_driver.hh"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace std::chrono_literals;
using namespace cybergear_m5_driver;


class CybeargearM5BridgeNode: public rclcpp::Node
{
  public:
    CybeargearM5BridgeNode()
    : Node("cybergear_m5_bridge_node")
    {
      publisher_ = create_publisher<sensor_msgs::msg::JointState>("joint_state", 1);
      timer_ = create_wall_timer(500ms, std::bind(&CybeargearM5BridgeNode::timer_callback, this));
      driver_ = std::make_shared<CyberGearM5Driver>();
      driver_->init("/dev/ttyACM0", 115200, {0x7F, 0x7E}, cybergear_m5_driver::ControlMode::CURRENT);
    }

  private:
    void timer_callback()
    {
      auto message = sensor_msgs::msg::JointState();
      publisher_->publish(message);
      driver_->control_current(0x7F, 0.1);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    CyberGearM5Driver::Ptr driver_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CybeargearM5BridgeNode>());
  rclcpp::shutdown();
  return 0;
}
