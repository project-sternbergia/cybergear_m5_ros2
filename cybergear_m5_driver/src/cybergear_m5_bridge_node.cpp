#include <chrono>
#include <functional>
#include <memory>
#include <ratio>
#include <algorithm>
#include <string>
#include <unordered_map>

#include "cybergear_m5_driver/cybergear_m5_driver.hh"
#include "cybergear_m5_driver/cybergear_m5_packet.hh"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace std::chrono_literals;
using namespace cybergear_m5_driver;


struct MotorConfig
{
  uint8_t id;                 //!< motor id
  ControlMode mode;           //!< control mode
  float velocity_limit;       //!< velocity limit (max: 30 [rad/s])
  float current_limit;        //!< current limit (max: 27 [A])
  float torque_limit;         //!< torque limit (max: 12 [Nm])
  float position_kp;          //!< position control gain for position control mode. (max: 200, default: 30.0)
  float velocity_kp;          //!< velocity control gain for velocity and position control mode. (max: 200, default: 2.0)
  float velocity_ki;          //!< velocity control gain for velocity and position control mode. (max: 200, default: 0.021)
};


class CybeargearM5BridgeNode: public rclcpp::Node
{
  public:
    typedef std::unordered_map<std::string, MotorCommand> MotorCommandMap;
    typedef std::unordered_map<std::string, MotorConfig> MotorConfigMap;

    CybeargearM5BridgeNode()
      : Node("cybergear_m5_bridge_node")
      , port_name_("/dev/ttyACM0")
      , baudrate_(115200)
      , frame_id_("body")
      , rate_(1000)
      , enable_on_start_(false)
      , disable_on_end_(true)
      , motor_command_map_()
      , motor_config_map_()
      , timer_()
      , js_pub_()
      , jc_sub_()
      , driver_()
    {
      // init parameters
      declare_parameter("port_name", port_name_);
      declare_parameter("baudrate", static_cast<int>(baudrate_));
      declare_parameter("frame_id", frame_id_);
      declare_parameter("enable_on_start", enable_on_start_);
      declare_parameter("disable_on_end", disable_on_end_);
      declare_parameter("rate", rate_);

      // declare motor configs
      declare_parameter("motors.names", std::vector<std::string>());
      auto motor_names = get_parameter("motors.names").as_string_array();
      for (auto name : motor_names) {
        declare_parameter("motors." + name + ".id", 0x00);
        declare_parameter("motors." + name + ".mode", "position");
        declare_parameter("motors." + name + ".velocity_limit", DEFAULT_VELOCITY_LIMIT);
        declare_parameter("motors." + name + ".current_limit", DEFAULT_CURRENT_LIMIT);
        declare_parameter("motors." + name + ".torque_limit", DEFAULT_TORQUE_LIMIT);
        declare_parameter("motors." + name + ".position_kp", DEFAULT_POSITION_KP);
        declare_parameter("motors." + name + ".velocity_kp", DEFAULT_VELOCITY_KP);
        declare_parameter("motors." + name + ".velocity_ki", DEFAULT_VELOCITY_KI);
      }

      // get parameters
      port_name_ = get_parameter("port_name").as_string();
      baudrate_ = get_parameter("baudrate").as_int();
      frame_id_ = get_parameter("frame_id").as_string();
      rate_ = get_parameter("rate").as_double();
      enable_on_start_ = get_parameter("enable_on_start").as_bool();
      disable_on_end_ = get_parameter("disable_on_end").as_bool();

      for (auto name : motor_names) {
        MotorConfig config;
        config.id = get_parameter("motors." + name + ".id").as_int();
        config.velocity_limit = get_parameter("motors." + name + ".velocity_limit").as_double();
        config.current_limit = get_parameter("motors." + name + ".current_limit").as_double();
        config.torque_limit = get_parameter("motors." + name + ".torque_limit").as_double();
        config.position_kp = get_parameter("motors." + name + ".position_kp").as_double();
        config.velocity_kp = get_parameter("motors." + name + ".velocity_kp").as_double();
        config.velocity_ki = get_parameter("motors." + name + ".velocity_ki").as_double();

        // get motor mode
        std::string mode = get_parameter("motors." + name + ".mode").as_string();
        std::transform(mode.begin(), mode.end(), mode.begin(), tolower);
        if (mode == "position") config.mode = ControlMode::Position;
        else if (mode == "velocity") config.mode = ControlMode::Velocity;
        else if (mode == "current") config.mode = ControlMode::Current;
        else if (mode == "motion") config.mode = ControlMode::Motion;
        else config.mode = ControlMode::Position;

        motor_config_map_[name] = config;
        RCLCPP_INFO(get_logger(), "Load [%s] motor config. id [0x%02x] control mode : [%s]", name.c_str(), config.id, mode.c_str());
      }

      for (auto const & [key, config] : motor_config_map_) {
        MotorCommand cmd = {rclcpp::Clock().now(), config.id, 0.0, 0.0, 0.0};
        motor_command_map_[key] = cmd;
      }

      // init pub/sub interfaces
      js_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
      jc_sub_ = create_subscription<sensor_msgs::msg::JointState>("joint_command", 1,
        std::bind(&CybeargearM5BridgeNode::joint_command_callback, this, std::placeholders::_1));

      // init timer callback
      auto duration = std::chrono::duration<int64_t, std::milli>(static_cast<int64_t>(1.0 / rate_ * 1000.0));
      timer_ = create_wall_timer(duration, std::bind(&CybeargearM5BridgeNode::timer_callback, this));

      // open cybergear m5 driver
      driver_ = std::make_shared<CyberGearM5Driver>();
      driver_->init(port_name_, baudrate_);

      // enable motors
      if (enable_on_start_) {
        for (auto const & [key, config] : motor_config_map_) {
          RCLCPP_INFO(get_logger(), "Set motor limit [0x%02x] speed : [%f], current : [%f], torque [%f] .", config.id, config.velocity_limit, config.current_limit, config.torque_limit);
          driver_->set_limit_speed(config.id, config.velocity_limit);
          driver_->set_limit_current(config.id, config.current_limit);
          driver_->set_limit_torque(config.id, config.torque_limit);

          RCLCPP_INFO(get_logger(), "Set motor control gain [0x%02x] pkp : [%f], vkp [%f], vki [%f].", config.id, config.position_kp, config.velocity_kp, config.velocity_ki);
          driver_->set_position_control_gain(config.id, config.position_kp);
          driver_->set_velocity_control_gain(config.id, config.velocity_kp, config.velocity_ki);

          RCLCPP_INFO(get_logger(), "Enable motor [0x%02x] in [%d] mode.", config.id, static_cast<int>(config.mode));
          driver_->enable_motor(config.id, config.mode);
        }
      }
    }

    virtual ~CybeargearM5BridgeNode()
    {
      if (disable_on_end_) {
        for (auto const & [key, config] : motor_config_map_) {
          driver_->reset_motor(config.id);
        }
      }
      driver_->shutdown();
    }

  private:
    void timer_callback()
    {
      auto msg = sensor_msgs::msg::JointState();

      // set data
      for (auto const & [key, config] : motor_config_map_) {
        if (config.mode == ControlMode::Position) {
          driver_->control_position(config.id, motor_command_map_[key].position);
        }
        else if (config.mode == ControlMode::Velocity) {
          driver_->control_velocity(config.id, motor_command_map_[key].velocity);
        }
        else if (config.mode == ControlMode::Current) {
          driver_->control_current(config.id, motor_command_map_[key].effort);

        } else if (config.mode == ControlMode::Motion) {
          driver_->control_motion(config.id, motor_command_map_[key].position,
            motor_command_map_[key].velocity, motor_command_map_[key].effort);
        }

        // get motor status
        MotorStatus mot;
        driver_->get_motor_status(config.id, mot);
        msg.name.push_back(key);
        msg.position.push_back(mot.position);
        msg.velocity.push_back(mot.velocity);
        msg.effort.push_back(mot.effort);
        msg.header.stamp = rclcpp::Clock().now();
        msg.header.frame_id = frame_id_;
      }

      js_pub_->publish(msg);
    }

    void joint_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
      for (size_t idx = 0; idx < msg->name.size(); ++idx) {
        std::string name = msg->name[idx];
        MotorCommand cmd;
        cmd.id = motor_config_map_[name].id;
        cmd.stamp = msg->header.stamp;
        cmd.position = (msg->position.size() > idx) ? msg->position[idx] : 0.0;
        cmd.velocity = (msg->velocity.size() > idx) ? msg->velocity[idx] : 0.0;
        cmd.effort = (msg->effort.size() > idx) ? msg->effort[idx] : 0.0;
        motor_command_map_[name] = cmd;
      }
    }

    // serial port settings
    std::string port_name_;               //!< serial device name (default: /dev/ttyACM0)
    uint32_t baudrate_;                   //!< serial port baudrate (default: 115200)
    std::string frame_id_;                //!< frame id for ros publisher
    double rate_;                         //!< publish rate (Hz)
    bool enable_on_start_;                //!< enable on start param
    bool disable_on_end_;                 //!< disable on end param
    MotorCommandMap motor_command_map_;   //!< motor command map
    MotorConfigMap motor_config_map_;     //!< motor config map

    // ros configs
    rclcpp::TimerBase::SharedPtr timer_;                                      //!< ros timer callback
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;       //!< joint state publisher for motor state
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jc_sub_;    //!< joint command subscriber for motor command
    CyberGearM5Driver::Ptr driver_;                                           //!< cybergear m5 driver object
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CybeargearM5BridgeNode>();
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
  }
  node.reset();
  rclcpp::shutdown();
  return 0;
}
