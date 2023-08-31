#ifndef NEO_LINEAR_AXIS__NLA_HARDWARE_INTERFACE_HPP_
#define NEO_LINEAR_AXIS__NLA_HARDWARE_INTERFACE_HPP_

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

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
  CallbackReturn on_activate(const hardware_interface::HardwareInfo & /*info*/);
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*info*/);
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  double joint_position_commands_;
  double joint_position_;
  double timeout_;
  double homeVel_;
  int homeDigIn_;
  double control_rate_;
  int heartbeat_divider_;
  int status_divider_;
  std::string canIface_;

  rclcpp::Clock clock_;
  
  hardware_interface::HardwareInfo info_;
  std::unique_ptr<NeoLinearJoint> linear_joint_;
  motor_t motor;

// Check for other functions that needs to be included and overrided. 
};

}


#endif // NEO_LINEAR_AXIS__NLA_HARDWARE_INTERFACE_HPP_