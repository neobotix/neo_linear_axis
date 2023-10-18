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

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include "rclcpp/rclcpp.hpp"

#include <neo_linear_axis/can_msg.hpp>
#include <neo_linear_axis/motor.hpp>
#include <neo_linear_axis/ElmoMotorModule.hpp>

#include <mutex>


#ifndef NEO_LINEAR_AXIS__NEO_LINEAR_JOINT_HPP_
#define NEO_LINEAR_AXIS__NEO_LINEAR_JOINT_HPP_

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
  void checkSocketError(bool & isError);
  bool inSafetyRegion();
  void getGpioData(int32_t & dig_1, int32_t & dig_2);

  // void sdoDownload(
  //   int & node_id, int32_t obj_index, int32_t obj_sub_index,
  //   int32_t data);
  // int32_t read_int32(const can_msg_t & msg, int offset);
  // float read_float(const can_msg_t & msg, int offset)
private:
  motor_t motor_;
  rclcpp::Clock clock_;

  rclcpp::Time last_update_time_;
  rclcpp::Time last_sync_time_;

  double motor_timeout_;
  std::shared_ptr<ElmoMotorModule> motorOp_;
  double homeVel_;
  double currPose_ = 0.0;
  double currVel_ = 0.0;
  
  double lookahead_time_ = 0.0; // hardcoding the values
  double targetPose_ = 0.0;
  const double pGain_ = 10.0;
  const double homingGain_ = 0.15;

  int syncCounter_;
  int heartbeatdivider_;
  int statusdivider_;
  rclcpp::Logger logger_;
  bool safety_shift_ = false;
  bool motor_ready_ = false;
  double last_command_ = 0.0;
  int iter = 0;
  bool goal_reached_ = false;
};

#endif // NEO_LINEAR_AXIS__NEO_LINEAR_JOINT_HPP_