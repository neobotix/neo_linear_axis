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

void NeoLinearJoint::setCommonMotorParams(
  double & control_rate,
  int & heartbeat_divider,
  int & status_divider)
{
  heartbeatdivider_ = heartbeat_divider;
  statusdivider_ = status_divider;
  motorOp_->setCommonMotorParams(control_rate, heartbeat_divider);
}

void NeoLinearJoint::initMotor()
{
  motorOp_->motorOff();
  motorOp_->stopMotion();

  // Disable watchdog
  motorOp_->setMotorModulo();
  motorOp_->enableVelMode(); // Just for Homing
  motorOp_->resetPosCounter();
  motorOp_->configurePDOMapping();
  motorOp_->motorOn();
  motorOp_->requestMotorStatus();

  /*show error if any in the above initialization step*/

}

void NeoLinearJoint::setClock(rclcpp::Clock & clock)
{
  clock_ = clock;
  motorOp_->setClock(clock);
}

void NeoLinearJoint::setMotorTimeout(double & timeout)
{
  motor_timeout_ = timeout;
}

void NeoLinearJoint::setHomeVel(double & homeVel)
{
  homeVel_ = homeVel;
  motorOp_->setHomeVel(homeVel_);
}

void NeoLinearJoint::setHomeDigIn(int & digIn)
{
  motorOp_->setHomeDigIn(digIn);
}

void NeoLinearJoint::updateMotor()
{
  // Check for motor timeouts

  if (motor_.state != ST_MOTOR_FAILURE) {
    motorOp_->checkMotorTimeout(motor_timeout_);
    if (motor_.state == ST_MOTOR_FAILURE) {
      std::cout<<"motor timeout"<<std::endl;
    }
  }

  // Check if motor is operational

  if (motorOp_->isReset()
    && motorOp_->isOperational()) {
    std::cout<<"Linear joint is operational"<<std::endl;
  }

  // Start homing if not homed

  if (!motorOp_->isHomed()) /**check if we need a sync counter later**/{
    motorOp_->setHomeVel(homeVel_);
    motorOp_->startHoming();
  }

  // Checking for homing status

  if (!motorOp_->isHoming()) {
    if (motorOp_->isOperational()) {
      std::cout<<"Homing interrupted"<<std::endl;
    } else if (motorOp_->isHomed()) {
      std::cout<<"Homing Finished"<<std::endl;
      motorOp_->finishHoming();
    }
  }

  double futurePose = 0.0;
  double poseDiff = 0.0;
  double ctrlVel = 0.0;

  if (currPose_ >= 2.0 || currPose_ <= 0.0) {
    // Engage brakes    

    // Set Motor Velocity 
    ctrlVel = 0.0;
    motorOp_->motorSetLinearJointVel(ctrlVel);
  } else {
    // Setting position and converting it to velocity commands
    futurePose = currPose_ + currVel_ * lookahead_time_;
    poseDiff = targetPose_ - futurePose;
    ctrlVel = poseDiff * steerGain_;
  }

  if (abs(ctrlVel) > 0) {
    // Disengage brakes - if engaged

    // Set ctrlVel
    motorOp_->motorSetLinearJointVel(ctrlVel); // Note : Convert rotational velocity to linear velocity
    motorOp_->beginMotion();  
  }
  // } else {
  //   // motorOp_->engageBrakes();
  //   continue;
  // }
  
  // Check for update timeout

  motorOp_->sendSyncMsg();
  motorOp_->sendHeartBeat();

  // Status request checker
  if (syncCounter_ % statusdivider_ == 0) {
        motorOp_->requestMotorStatus();    // request status update
  }

  if (syncCounter_ % heartbeatdivider_ == 0) {
    motorOp_->sendHeartBeat(); // Heartbeat of master to slave
  }
  // Heartbeat checker

  syncCounter_++;
}