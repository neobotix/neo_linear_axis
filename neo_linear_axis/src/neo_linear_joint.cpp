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
{
}

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
  motorOp_->enablePosMode();  // Just for Homing
  motorOp_->resetPosCounter();
  motorOp_->configurePDOMapping();
  motorOp_->requestMotorStatus();
  motorOp_->readIOs();
  motorOp_->motorOn();
  RCLCPP_INFO(logger_, "Initializing done");
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

void NeoLinearJoint::getGpioData(
  int32_t & dig_1,
  int32_t & dig_2)
{
  motorOp_->digInStates(dig_1, dig_2);
}

void NeoLinearJoint::stopMotion()
{
  motorOp_->stopMotion();
  motorOp_->motorOff();
}

void NeoLinearJoint::setEMState(bool EMState)
{
  EMState_ = EMState;
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

void NeoLinearJoint::getJointVel(double & joint_velocity)
{
  joint_velocity = motorOp_->currVel();
}

// sets the linear position of the joint
void NeoLinearJoint::setJointPos(double & joint_position)
{
  targetPose_ = joint_position;
  // ToDo: Forward the joint position to the motor controllers
}

bool NeoLinearJoint::inSafetyRegion()
{
  if (safety_shift_) {
    return true;
  }
  return false;
}

// sets the initial position of the axis after homing

void NeoLinearJoint::setInitPos(double & init_pos)
{
  init_pos_ = init_pos;
}

// sets the linear joint velocity

void NeoLinearJoint::setJointVel(double & joint_velocity)
{
  linearVel_ = joint_velocity;
  if (abs(linearVel_) >= 0.006 && !motorOp_->isOperational()) {
    motorOp_->motorOn();
  }
  if (abs(linearVel_) < 0.006 && motorOp_->isOperational()) {
    motorOp_->motorOff();
  }
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

  if (motorOp_->isReset() &&
    motorOp_->isOperational())
  {
    motor_ready_ = true;
    motorOp_->motorOff();
    motorOp_->sendSyncMsg();
    motorOp_->requestMotorStatus();
    RCLCPP_INFO(logger_, "Linear joint is operational");
  }

  // Start homing if not homed

  if (!motorOp_->isHomed() && !EMState_) {
    if (!motorOp_->isOperational()) {
      motorOp_->motorOn();
    }
    motorOp_->setHomeVel(homeVel_);
    motorOp_->startHoming();
    motorOp_->readIOs();
    motorOp_->requestMotorStatus();
  }

  // Checking for homing status

  if (motorOp_->isHoming()) {
    if (!motorOp_->isOperational()) {
      RCLCPP_WARN(logger_, "Homing interrupted");
      motorOp_->setIsHoming(EMState_);
    } else if (motorOp_->checkHomingDone()) {
      RCLCPP_INFO(logger_, "Homing finished");
      motorOp_->finishHoming();
    } else {
      motorOp_->reqHomStatus();
    }
  }

  // Motor ready after homing. then ready to get in position commands
  if (motor_ready_) {
    if (!safety_shift_ &&
      motor_on_ &&
      motorOp_->isOperational())
    {
      if (!command_sent) {
        RCLCPP_INFO(logger_, "sending to safety region");
        motorOp_->motorSetPos(init_pos_);
        motorOp_->beginMotion();
        command_sent = true;
      }
      if (abs(currPose_ - init_pos_) <= 0.001) {
        safety_shift_ = true;
        motorOp_->motorOff();
        RCLCPP_INFO(logger_, "Reached safety region");
        RCLCPP_INFO(logger_, "Ready to send position commands");
      }
    }

    if (safety_shift_) {
      if (motorOp_->isOperational()) {
        motorOp_->motorSetVel(linearVel_);
        motorOp_->beginMotion();
      }
    }

    syncCounter_++;
    motorOp_->sendSyncMsg();

    last_sync_time_ = rclcpp::Clock().now();
    motorOp_->setSyncTime(last_sync_time_);

    if (!safety_shift_ && !motorOp_->isOperational()) {
      motorOp_->motorOn();
      motor_on_ = true;
    }

    // Status request checker
    if (syncCounter_ % statusdivider_ == 0) {
      motorOp_->requestMotorStatus();        // request status update
    }

    if ((syncCounter_ + 1) % heartbeatdivider_ == 0) {
      motorOp_->sendHeartBeat();  // Heartbeat of master to slave
      motorOp_->readIOs();
    }
  }
}
