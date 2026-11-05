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

#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <neo_linear_axis/nla_hardware_interface.hpp>

using std::placeholders::_1;

namespace nla
{

NlaHardwareInterface::NlaHardwareInterface()
{
  helperNode_ = std::make_shared<rclcpp::Node>("linear_axis_helper_node");
  sub_emergency_stop_ = helperNode_->create_subscription<neo_msgs2::msg::EmergencyStopState>(
      "emergency_stop_state", 1, std::bind(
        &NlaHardwareInterface::emergency_stop_callback, this,
        _1));
}

NlaHardwareInterface::~NlaHardwareInterface()
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  shutdown_hardware();
}

CallbackReturn NlaHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.hardware_parameters.count("can_iface") > 0) {
    canIface_ = info_.hardware_parameters.at("can_iface");
    RCLCPP_INFO(get_logger(), "can_iface for the system is set to %s", canIface_.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "can_iface not specified in the URDF. Defaults to can0");
  }

  if (info_.hardware_parameters.count("timeout") > 0) {
    timeout_ = std::stod(info_.hardware_parameters.at("timeout"));
    RCLCPP_INFO(get_logger(), "timeout_ for motor is set to %f", timeout_);
  } else {
    RCLCPP_WARN(get_logger(), "motor timeout not specified in the URDF. Defaults to 0.2 s");
  }

  if (info_.hardware_parameters.count("control_rate") > 0) {
    control_rate_ = std::stod(info_.hardware_parameters.at("control_rate"));
    RCLCPP_INFO(get_logger(), "control_rate_ for motor is set to %f", control_rate_);
  } else {
    RCLCPP_WARN(get_logger(), "control_rate not specified in the URDF. Defaults to 100.0 Hz");
  }

  if (info_.hardware_parameters.count("initial_pose") > 0) {
    init_pos_ = std::stod(info_.hardware_parameters.at("initial_pose"));
    RCLCPP_INFO(get_logger(), "Setting the inital pose to %f", init_pos_);
  } else {
    RCLCPP_WARN(get_logger(), "inital pose not specified in the URDF. Defaults to 0.1 m");
  }

  if (info_.hardware_parameters.count("home_dig_in") > 0) {
    homeDigIn_ = std::stoi(info_.hardware_parameters.at("home_dig_in"));
    RCLCPP_INFO(get_logger(), "Setting the home dig in to %d", homeDigIn_);
  } else {
    RCLCPP_WARN(get_logger(), "home dig in not specified in the URDF. Defaults to 5");
  }

  if (!std::isfinite(control_rate_) || control_rate_ <= 0.0) {
    RCLCPP_ERROR(get_logger(), "control_rate must be finite and positive");
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn NlaHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*info*/)
{
  // A new configuration starts a new session; deactivation keeps this joint intact.
  motor_initialized_ = false;
  active_ = false;
  linear_joint_ = std::make_unique<NeoLinearJoint>(motor, get_logger());
  linear_joint_->setInitPos(init_pos_);
  linear_joint_->setClock(clock_);
  linear_joint_->setMotorTimeout(timeout_);
  linear_joint_->setHomeVel(homeVel_);
  linear_joint_->setHomeDigIn(homeDigIn_);
  linear_joint_->setCommonMotorParams(
    control_rate_,
    heartbeat_divider_,
    status_divider_);  // Setting dummy for now
  linear_joint_->setCanIface(canIface_);  // Setting dummy for now

  return configure_communication();
}

CallbackReturn NlaHardwareInterface::configure_communication()
{
  try {
    communication_failed_.store(false);
    linear_joint_->setEMState(emstop_);
    if (!linear_joint_->isSocketOpened()) {
      bool socketError = false;
      linear_joint_->openSocket();
      linear_joint_->checkSocketError(socketError);
      if (socketError) {
        return CallbackReturn::ERROR;
      }
      linear_joint_->initComms();
      rclcpp::Rate loop_rate(10);  // Allow communication initialization to settle.
      loop_rate.sleep();
    }
    // Keep the configured but inactive drive disabled, with heartbeat monitoring suspended.
    linear_joint_->stopMotion();
    receive_running_.store(true);
    read_thread_ = std::thread(&NlaHardwareInterface::receive_loop, this);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize CAN communication: %s", ex.what());
    shutdown_hardware();
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn NlaHardwareInterface::on_cleanup(const rclcpp_lifecycle::State & /*info*/)
{
  return shutdown_hardware();
}

CallbackReturn NlaHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  bool socket_error = false;
  if (linear_joint_) {
    linear_joint_->checkSocketError(socket_error);
  }
  if (!linear_joint_ || !linear_joint_->isSocketOpened() || socket_error ||
    communication_failed_.load() || !receive_running_.load())
  {
    RCLCPP_ERROR(get_logger(), "CAN communication is unavailable; configure the hardware again");
    return CallbackReturn::ERROR;
  }

  try {
    rclcpp::spin_some(helperNode_);
    linear_joint_->setEMState(emstop_);
    set_command(info_.joints[0].name + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
    if (!motor_initialized_) {
      RCLCPP_INFO(get_logger(), "Initializing motor for the first activation");
      linear_joint_->initMotor();
      motor_initialized_ = true;
    } else {
      RCLCPP_INFO(get_logger(), "Re-enabling motor with its existing home reference");
      linear_joint_->motorOn();
    }
    active_ = true;
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Failed to activate linear axis: %s", ex.what());
    shutdown_hardware();
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn NlaHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*info*/)
{
  active_ = false;
  RCLCPP_INFO(get_logger(), "Stopping and disabling motor; CAN communication remains active");
  try {
    set_command(info_.joints[0].name + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
    linear_joint_->stopMotion();
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Failed to deactivate linear axis: %s", ex.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn NlaHardwareInterface::on_shutdown(const rclcpp_lifecycle::State & /*info*/)
{
  return shutdown_hardware();
}

CallbackReturn NlaHardwareInterface::on_error(const rclcpp_lifecycle::State & /*info*/)
{
  RCLCPP_INFO(get_logger(), "Error received, stopping the motor");
  return shutdown_hardware();
}

CallbackReturn NlaHardwareInterface::shutdown_hardware()
{
  active_ = false;
  motor_initialized_ = false;
  receive_running_.store(false);
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
  if (linear_joint_) {
    try {
      linear_joint_->shutdown();
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Failed to shut down linear axis: %s", ex.what());
      return CallbackReturn::ERROR;
    }
  }
  return CallbackReturn::SUCCESS;
}

void NlaHardwareInterface::receive_loop()
{
  // Send request for reading the motor data
  while (receive_running_.load() && rclcpp::ok()) {
    try {
      linear_joint_->readMotor();
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "CAN receive failed: %s", ex.what());
      communication_failed_.store(true);
      break;
    }
  }
  receive_running_.store(false);
}

return_type NlaHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (communication_failed_.load()) {
    return return_type::ERROR;
  }
  int32_t gpio_states[2] = {0, 0};
  double joint_position = 0.0;
  double joint_velocity = 0.0;

  linear_joint_->getJointPos(joint_position);
  linear_joint_->getJointVel(joint_velocity);
  linear_joint_->getGpioData(gpio_states[0], gpio_states[1]);

  // Use Jazzy set_state() API with fully qualified interface names
  set_state(info_.joints[0].name + "/" + hardware_interface::HW_IF_POSITION, joint_position);
  set_state(info_.joints[0].name + "/" + hardware_interface::HW_IF_VELOCITY, joint_velocity);

  set_state(info_.gpios[0].name + "/dig_in_1", static_cast<double>(gpio_states[0]));
  set_state(info_.gpios[0].name + "/dig_in_2", static_cast<double>(gpio_states[1]));

  rclcpp::spin_some(helperNode_);  // for reading emstop

  return return_type::OK;
}

return_type NlaHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (communication_failed_.load()) {
    return return_type::ERROR;
  }
  if (!active_) {
    return return_type::OK;
  }
  try {
    const std::string vel_cmd_name = info_.joints[0].name + "/" +
      hardware_interface::HW_IF_VELOCITY;
    if (emstop_) {
      set_command(vel_cmd_name, 0.0);
      linear_joint_->stopMotion();
      return return_type::OK;
    }
    // Pass zero commands too, so an earlier velocity cannot remain in effect.
    if (linear_joint_->inSafetyRegion()) {
      double joint_velocity_commands = get_command(vel_cmd_name);
      linear_joint_->setJointVel(joint_velocity_commands);
    }
    linear_joint_->updateMotor();
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Failed to write linear axis command: %s", ex.what());
    return return_type::ERROR;
  }

  return return_type::OK;
}

void NlaHardwareInterface::emergency_stop_callback(
  const neo_msgs2::msg::EmergencyStopState::SharedPtr state)
{
  if (state->emergency_state == neo_msgs2::msg::EmergencyStopState::EMSTOP) {
    emstop_ = true;
    linear_joint_->setEMState(emstop_);
  }
  if (state->emergency_state == neo_msgs2::msg::EmergencyStopState::EMFREE) {
    emstop_ = false;
    linear_joint_->setEMState(emstop_);
  }
}

}  // namespace nla

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  nla::NlaHardwareInterface, hardware_interface::SystemInterface)
