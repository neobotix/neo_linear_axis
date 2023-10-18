#ifndef NEO_LINEAR_AXIS__NLA_HARDWARE_INTERFACE_HPP_
#define NEO_LINEAR_AXIS__NLA_HARDWARE_INTERFACE_HPP_

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/visibility_control.h"

#include "neo_linear_axis/motor.hpp"
#include "neo_linear_axis/neo_linear_joint.hpp"

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace nla {

class NlaHardwareInterface : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & /*info*/) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & /*info*/) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*info*/) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;
  rclcpp::Logger logger_{rclcpp::get_logger("neo_linear_axis")};
  NlaHardwareInterface();
  ~NlaHardwareInterface();
  void receive_loop();

private:
  double joint_position_commands_ = 0.0;
  double joint_velocity_commands_ = 0.0;
  double joint_position_ = 0.0;
  double joint_velocity_ = 0.0;
  double timeout_ = 0.2;
  double homeVel_ = 0.01;
  int homeDigIn_ = 9;
  double control_rate_ = 100.0;
  int heartbeat_divider_ = 5;
  int status_divider_ = 10;
  std::string canIface_ = "can0";
  std::mutex node_mutex_;
  std::thread read_thread_;

  rclcpp::Clock clock_;

  std::unique_ptr<NeoLinearJoint> linear_joint_;
  std::vector<double> gpio_states_;
  motor_t motor;

// Check for other functions that needs to be included and overrided. 
};

}


#endif // NEO_LINEAR_AXIS__NLA_HARDWARE_INTERFACE_HPP_