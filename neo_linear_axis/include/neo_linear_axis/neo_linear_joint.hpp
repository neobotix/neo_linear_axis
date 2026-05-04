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

#ifndef NEO_LINEAR_AXIS__NEO_LINEAR_JOINT_HPP_
#define NEO_LINEAR_AXIS__NEO_LINEAR_JOINT_HPP_

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include <neo_linear_axis/ElmoMotorModule.hpp>
#include <neo_linear_axis/can_msg.hpp>
#include <neo_linear_axis/motor.hpp>

class NeoLinearJoint
{
public:
  // Defining constructor, later will be updated with args
  NeoLinearJoint(motor_t motor, rclcpp::Logger logger);
  ~NeoLinearJoint();

  void initComms();
  void initMotor();

  void setClock(rclcpp::Clock & clock);
  void setMotorTimeout(double & timeout);
  void updateMotor();
  void setHomeVel(double & homeVel);
  void setHomeDigIn(int & digin);
  void setCommonMotorParams(
    double & control_rate,
    int & heartbeat_divider,
    int & status_divider
  );
  bool isSocketOpened();
  void openSocket();
  void setCanIface(std::string & canIface);
  void readMotor();
  void shutdown();
  void getJointPos(double & joint_position);
  void getJointVel(double & joint_velocity);
  void setJointPos(double & joint_position);
  void setJointVel(double & joint_velocity);
  void setInitPos(double & init_pos);
  void checkSocketError(bool & isError);
  bool inSafetyRegion();
  void getGpioData(int32_t & dig_1, int32_t & dig_2);
  void stopMotion();
  void motorOff();
  void setEMState(bool EMState);

private:
  motor_t motor_;
  rclcpp::Clock clock_;

  rclcpp::Time last_update_time_;
  rclcpp::Time last_sync_time_;

  rclcpp::Logger logger_;

  double motor_timeout_ = 0.2;
  double homeVel_ = -0.03;
  double currPose_ = 0.0;
  double currVel_ = 0.0;
  double targetPose_ = 0.0;
  double linearVel_ = 0.0;
  double init_pos_ = 0.1;

  int syncCounter_ = 0;
  int heartbeatdivider_ = 0;
  int statusdivider_ = 0;

  bool safety_shift_ = false;
  bool motor_ready_ = false;

  bool command_sent = false;
  bool motor_on_ = false;

  bool EMState_ = true;
  std::shared_ptr<ElmoMotorModule> motorOp_;
};

#endif  // NEO_LINEAR_AXIS__NEO_LINEAR_JOINT_HPP_
