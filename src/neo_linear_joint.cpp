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

#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <neo_linear_axis/neo_linear_joint.hpp>

using std::placeholders::_1;

NeoLinearJoint::NeoLinearJoint(motor_t motor)
{
  motor_ = motor;
  motorOp_ = std::make_shared<ElmoMotorModule>(motor_);
}

NeoLinearJoint::~NeoLinearJoint()
{}

void NeoLinearJoint::initComms()
{
  motorOp_->initializeNmt();
}

void NeoLinearJoint::initMotor()
{
  motorOp_->motorOff();
  motorOp_->setMotorModulo();
  motorOp_->resetPosCounter();
  motorOp_->configurePDOMapping();
  motorOp_->motorOn();
  motorOp_->requestMotorStatus();

  /*show error if any in the above initialization step*/

}