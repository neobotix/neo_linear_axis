#ifndef NEO_LINEAR_AXIS__NLA_HARDWARE_INTERFACE_HPP_
#define NEO_LINEAR_AXIS__NLA_HARDWARE_INTERFACE_HPP_

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "neo_linear_axis/motor.hpp"
#include "neo_linear_axis/neo_linear_joint.hpp"

#include "neo_msgs2/msg/emergency_stop_state.hpp"

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace nla {

class NlaHardwareInterface : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & /*info*/) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & /*info*/) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*info*/) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & /*info*/) override;

  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;
  NlaHardwareInterface();
  ~NlaHardwareInterface();
  void receive_loop();

private:
  double timeout_ = 0.2;
  double homeVel_ = -0.03;
  int homeDigIn_ = 5;
  double control_rate_ = 100.0;
  double init_pos_ = 0.1;

  int heartbeat_divider_ = 5;
  int status_divider_ = 10;
  std::string canIface_ = "can0";
  std::mutex node_mutex_;
  std::thread read_thread_;

  rclcpp::Clock clock_;
  rclcpp::Node::SharedPtr helperNode_;

  std::unique_ptr<NeoLinearJoint> linear_joint_;
  motor_t motor;
  bool emstop_ = true;

  rclcpp::Subscription<neo_msgs2::msg::EmergencyStopState>::SharedPtr sub_emergency_stop_;
  void emergency_stop_callback(const neo_msgs2::msg::EmergencyStopState::SharedPtr state);

// Check for other functions that needs to be included and overrided. 
};

}


#endif // NEO_LINEAR_AXIS__NLA_HARDWARE_INTERFACE_HPP_
