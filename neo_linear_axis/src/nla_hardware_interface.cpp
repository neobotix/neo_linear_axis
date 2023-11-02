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

NlaHardwareInterface::NlaHardwareInterface(){}

NlaHardwareInterface::~NlaHardwareInterface(){
    RCLCPP_INFO(logger_, "Shutting down");
  linear_joint_->shutdown();
}

CallbackReturn NlaHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  if (info_.hardware_parameters.count("can_iface") > 0) {
    canIface_ = info_.hardware_parameters.at("can_iface");
    RCLCPP_INFO(logger_, "can_iface for the system is set to %s", canIface_.c_str());
  } else {
    RCLCPP_WARN(logger_, "can_iface not specified in the URDF. Defaults to can0");
  }

  if (info_.hardware_parameters.count("timeout") > 0) {
    timeout_ = std::stod(info_.hardware_parameters.at("timeout"));
    RCLCPP_INFO(logger_, "timeout_ for motor is set to %f", timeout_);
  } else {
    RCLCPP_WARN(logger_, "motor timeout not specified in the URDF. Defaults to 0.2 s");
  }

  if (info_.hardware_parameters.count("control_rate") > 0) {
    control_rate_ = std::stod(info_.hardware_parameters.at("control_rate"));
    RCLCPP_INFO(logger_, "control_rate_ for motor is set to %f", control_rate_);
  } else {
    RCLCPP_WARN(logger_, "control_rate not specified in the URDF. Defaults to 100.0 Hz");
  }

  if (info_.hardware_parameters.count("initial_pose") > 0) {
    init_pos_ = std::stod(info_.hardware_parameters.at("initial_pose"));
    RCLCPP_INFO(logger_, "Setting the inital pose to %f", init_pos_);
  } else {
    RCLCPP_WARN(logger_, "inital pose not specified in the URDF. Defaults to 0.1 m");
  }

  // Get all the necessary parameters for the motors
  linear_joint_ = std::make_unique<NeoLinearJoint>(motor, logger_);
  linear_joint_->setInitPos(init_pos_);
  linear_joint_->setClock(clock_);
  linear_joint_->setMotorTimeout(timeout_);
  linear_joint_->setHomeVel(homeVel_);
  linear_joint_->setHomeDigIn(homeDigIn_);
  linear_joint_->setCommonMotorParams(
    control_rate_,
    heartbeat_divider_,
    status_divider_); // Setting dummy for now
  linear_joint_->setCanIface(canIface_); // Setting dummy for now

  gpio_states_.resize(
    info_.gpios[0].state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());

  return CallbackReturn::SUCCESS;
}

CallbackReturn NlaHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*info*/)
{
  if (!linear_joint_->isSocketOpened()) {
    // Comms initialization
    bool socketError;
    linear_joint_->openSocket();
    linear_joint_->checkSocketError(socketError);
    if (socketError) {
      return CallbackReturn::ERROR;
    }
    linear_joint_->initComms();
  }
  return CallbackReturn::SUCCESS; 
}

CallbackReturn NlaHardwareInterface::on_activate(const rclcpp_lifecycle::State & )
{
  // Starting the motors
  RCLCPP_INFO(logger_, "Initializing motors");
  linear_joint_->initMotor();
  read_thread_ = std::thread(&NlaHardwareInterface::receive_loop, this);
  return CallbackReturn::SUCCESS;
}

CallbackReturn NlaHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*info*/)
{
  RCLCPP_INFO(logger_, "Shutting down");
  linear_joint_->shutdown();
  return CallbackReturn::SUCCESS;
}

CallbackReturn NlaHardwareInterface::on_error(const rclcpp_lifecycle::State & /*info*/)
{
  RCLCPP_INFO(logger_, "Error received, stopping the motor");
  linear_joint_->stopMotion();
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> NlaHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(4);

  // For now just handling positions, later will integrate the velocity and the torque interface

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, hardware_interface::HW_IF_POSITION, &joint_position_
    ));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &joint_velocity_
    ));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.gpios[0].name, "dig_in_" + std::to_string(1), &gpio_states_[0]
    ));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.gpios[0].name, "dig_in_" + std::to_string(2), &gpio_states_[1]
    ));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> NlaHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(1);

  // For now just handling velocities, later will integrate the position and the torque interface

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &joint_velocity_commands_));

  return command_interfaces;
}

void NlaHardwareInterface::receive_loop()
{
  // Send request for reading the motor data
  while(rclcpp::ok())
  {
    linear_joint_->readMotor();
  }
}

return_type NlaHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  RCLCPP_INFO_ONCE(logger_, "Reading motors");
  int32_t gpio_states[2] = {0, 0};

  linear_joint_->getJointPos(joint_position_);
  linear_joint_->getJointVel(joint_velocity_);
  linear_joint_->getGpioData(gpio_states[0], gpio_states[1]);

  gpio_states_[0] = static_cast<double>(gpio_states[0]);
  gpio_states_[1] = static_cast<double>(gpio_states[1]);

  return return_type::OK;
}

return_type NlaHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{

  RCLCPP_INFO_ONCE(logger_, "writing motors");

  // Joint velocities are set only after homing is done
  if (linear_joint_->inSafetyRegion()) {
    linear_joint_->setJointVel(joint_velocity_commands_);
  }

  // Send commands to motor commands
  linear_joint_->updateMotor();

  return return_type::OK;
}

} // namespace nla

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  nla::NlaHardwareInterface, hardware_interface::SystemInterface)
