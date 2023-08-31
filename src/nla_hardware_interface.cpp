/*********************************************************************
MIT License

Copyright (c) 2023 neobotix gmbh

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 *********************************************************************/

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <neo_linear_axis/nla_hardware_interface.hpp>

namespace nla {

CallbackReturn NlaHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  info_ = info;
  
  // Get all the necessary parameters for the motors
  linear_joint_ = std::make_unique<NeoLinearJoint>(motor);
  linear_joint_->setClock(clock_);
  linear_joint_->setMotorTimeout(timeout_);
  linear_joint_->setHomeVel(homeVel_);
  linear_joint_->setHomeDigIn(homeDigIn_);
  linear_joint_->setCommonMotorParams(
    control_rate_,
    heartbeat_divider_,
    status_divider_); // Setting dummy for now
  linear_joint_->setCanIface(canIface_); // Setting dummy for now

  return CallbackReturn::SUCCESS;
}

CallbackReturn NlaHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*info*/)
{
  // Comms initialization
  linear_joint_->initComms();
  return CallbackReturn::SUCCESS; 
}

CallbackReturn NlaHardwareInterface::on_activate(const hardware_interface::HardwareInfo & /*info*/)
{
  linear_joint_->initMotor();
  return CallbackReturn::SUCCESS;
}


CallbackReturn NlaHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*info*/)
{
  return CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> NlaHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(1);

  // For now just handling positions, later will integrate the velocity and the torque interface

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, hardware_interface::HW_IF_POSITION, &joint_position_
    ));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> NlaHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(1);

  // For now just handling positions, later will integrate the velocity and the torque interface

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[0].name, hardware_interface::HW_IF_POSITION, &joint_position_commands_));

  return command_interfaces;
}

return_type NlaHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!linear_joint_->isSocketOpened()) {
    linear_joint_->openSocket();
  }
  return return_type::OK;
}

return_type NlaHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  return return_type::OK;
}

} // namespace nla

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  nla::NlaHardwareInterface, hardware_interface::SystemInterface)
