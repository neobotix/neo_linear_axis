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

#ifndef NEO_LINEAR_AXIS__NLA_HARDWARE_INTERFACE_HPP_
#define NEO_LINEAR_AXIS__NLA_HARDWARE_INTERFACE_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "neo_linear_axis/motor.hpp"
#include "neo_linear_axis/neo_linear_joint.hpp"

#include "neo_msgs2/msg/emergency_stop_state.hpp"

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace nla
{

class NlaHardwareInterface : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
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
  void stop_receive_thread();

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
  std::atomic<bool> receive_running_{false};

  rclcpp::Clock clock_;
  rclcpp::Node::SharedPtr helperNode_;

  std::unique_ptr<NeoLinearJoint> linear_joint_;
  motor_t motor;
  bool emstop_ = true;

  rclcpp::Subscription<neo_msgs2::msg::EmergencyStopState>::SharedPtr sub_emergency_stop_;
  void emergency_stop_callback(const neo_msgs2::msg::EmergencyStopState::SharedPtr state);

// Check for other functions that needs to be included and overrided.
};

}  // namespace nla

#endif  // NEO_LINEAR_AXIS__NLA_HARDWARE_INTERFACE_HPP_
