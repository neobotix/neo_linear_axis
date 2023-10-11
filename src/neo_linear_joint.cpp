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
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <neo_linear_axis/neo_linear_joint.hpp>

using std::placeholders::_1;

NeoLinearJoint::NeoLinearJoint(motor_t motor, rclcpp::Logger Logger)
  :motor_(motor), logger_(Logger)
{
  motorOp_ = std::make_shared<ElmoMotorModule>(motor_, Logger);
}

NeoLinearJoint::~NeoLinearJoint()
{}

void NeoLinearJoint::initComms()
{
  RCLCPP_INFO(logger_, "Initializing communication");
  motorOp_->initializeNmt();
  ::usleep(100 * 1000);
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
  motorOp_->setDigitalInput();

  /*show error if any in the above initialization step*/

}

void NeoLinearJoint::setCanIface(std::string & canIface)
{
  motorOp_->setCanIface(canIface);
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

bool NeoLinearJoint::isSocketOpened()
{
 return motorOp_->isSocketOpened();
}

void NeoLinearJoint::openSocket()
{
  motorOp_->openSocket();
}

void NeoLinearJoint::checkSocketError(bool & isError)
{
  motorOp_->checkSocketError(isError);
}

void NeoLinearJoint::readMotor()
{
  motorOp_->readMotor();
}

void NeoLinearJoint::shutdown()
{
  motorOp_->shutdown();
}

// Gives back the linear position of the joint
void NeoLinearJoint::getJointPos(double & joint_position)
{
  currPose_ = motorOp_->getPose();
  last_update_time_ = rclcpp::Clock().now();
  joint_position = currPose_;
}

// sets the linear position of the joint
void NeoLinearJoint::setJointPos(double & joint_position)
{
  targetPose_ = joint_position ;
}

void NeoLinearJoint::updateMotor()
{
  // Check for motor timeouts

  if (motor_.state != ST_MOTOR_FAILURE) {
    motorOp_->checkMotorTimeout(motor_timeout_);
    if (motor_.state == ST_MOTOR_FAILURE) {
      RCLCPP_ERROR(logger_, "motor timeout");
    }
  }

  // Check if motor is operational

  if (motorOp_->isReset()
    && motorOp_->isOperational()) {
    RCLCPP_INFO(logger_, "Linear joint is operational");
  }

  // Start homing if not homed

  if (!motorOp_->isHomed()) /**check if we need a sync counter later**/{
    motorOp_->setHomeVel(homeVel_);
    motorOp_->startHoming();
  }

  // Checking for homing status

  if (!motorOp_->isHoming()) {
    if (motorOp_->isOperational()) {
      RCLCPP_WARN(logger_, "Homing interrupted");
    } else if (motorOp_->isHomed()) {
      RCLCPP_INFO(logger_, "Homing finished");
      motorOp_->finishHoming();
      motorOp_->motorOff();
    }
  }

  double futurePose = 0.0;
  double poseDiff = 0.0;
  double ctrlVel = 0.0;
  double linearVel = 0.0;
  currVel_ = motorOp_->getVel();

  if (currPose_ >= 2.0 || currPose_ <= 0.0) {
    // Engage brakes    

    // Set Motor Velocity 
    linearVel = 0.0;
    motorOp_->motorSetVel(linearVel);
  } else {
    // Setting position and converting it to velocity commands
    futurePose = currPose_ + currVel_ * lookahead_time_;
    poseDiff = targetPose_ - futurePose;
    linearVel = poseDiff * pGain_;
  }

  if (abs(linearVel) > 0.000 || currPose_ != targetPose_) {
    // Turning on the motor disengages the brake
    if (!motorOp_->isOperational()) {
      motorOp_->motorOn();
    } else {
      RCLCPP_INFO(logger_, "Motor is already switched on");
    }

    // Set ctrlVel
    ctrlVel = linearVel;

    motorOp_->motorSetVel(ctrlVel);
    motorOp_->beginMotion();  
    syncCounter_++;

    // Check for update timeout
    if(last_update_time_ < last_sync_time_) {
      if (motorOp_->isHomed()) {
        // warn
        RCLCPP_WARN(logger_, "Sync update timeout");
      }
    }

    motorOp_->sendSyncMsg();
    motorOp_->sendHeartBeat();

    last_sync_time_ = rclcpp::Clock().now();
    motorOp_->setSyncTime(last_sync_time_);

    // Status request checker
    if (syncCounter_ % statusdivider_ == 0) {
          motorOp_->requestMotorStatus();    // request status update
    }

    if (syncCounter_ % heartbeatdivider_ == 0) {
      motorOp_->sendHeartBeat(); // Heartbeat of master to slave
    }
    // Heartbeat checker

  } else {
    if (motorOp_->isOperational()) {
      motorOp_->motorOff();  
    } else {
      std::cout<<"Motor is already switched off"<<std::endl;
    }
    
  }
  
}