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

#include "rclcpp/rclcpp.hpp"
#include <neo_linear_axis/ElmoMotorModule.hpp>

using std::placeholders::_1;

ElmoMotorModule::ElmoMotorModule(motor_t & motor)
{
  // configuring CAN for the motor transmission
  motor_ = motor;
  motor_.can_Tx_PDO1 = motor_.node_id + 0x180;
  motor_.can_Tx_PDO2 = motor_.node_id + 0x280;
  motor_.can_Rx_PDO2 = motor_.node_id + 0x300;
  motor_.can_Tx_SDO = motor_.node_id + 0x580;
  motor_.can_Rx_SDO = motor_.node_id + 0x600;
}

void ElmoMotorModule::initializeNmt()
{
  waitForCanSock_ = true;

  // start network
  {
    can_msg_t msg;
    msg.id = 0;
    msg.length = 2;
    msg.data[0] = 1;
    msg.data[1] = 0;
    transmitCan(msg);
  }
  canSync();
}

void ElmoMotorModule::configurePDOMapping()
{
  // stop all emissions of TPDO1
  canOpenSDODownload(0x1A00, 0, 0);

  // position 4 byte of TPDO1
  canOpenSDODownload(0x1A00, 1, 0x60640020);

  // velocity 4 byte of TPDO1
  canOpenSDODownload(0x1A00, 2, 0x60690020);

  canSync();

  // transmission type "synch"
  canOpenSDODownload(0x1800, 2, 1);

  // activate mapped objects
  canOpenSDODownload(0x1A00, 0, 2);

  canSync();
}

void ElmoMotorModule::canOpenSDODownload(
  int32_t obj_index,
  int32_t obj_sub_index,
  int32_t data)
{
  const int32_t ciInitDownloadReq = 0x20;
  const int32_t ciNrBytesNoData = 0x00;
  const int32_t ciExpedited = 0x02;
  const int32_t ciDataSizeInd = 0x01;

  can_msg_t msg;
  msg.id = motor_.can_Rx_SDO;
  msg.length = 8;
  msg.data[0] = ciInitDownloadReq | (ciNrBytesNoData << 2) | ciExpedited | ciDataSizeInd;
  msg.data[1] = obj_index;
  msg.data[2] = obj_index >> 8;
  msg.data[3] = obj_sub_index;
  msg.data[4] = data;
  msg.data[5] = data >> 8;
  msg.data[6] = data >> 16;
  msg.data[7] = data >> 24;
  transmitCan(msg);
}

void ElmoMotorModule::transmitCan(const can_msg_t & msg)
{
  ::can_frame out = {};
  out.can_id = msg.id;
  out.can_dlc = msg.length;
  for (int i = 0; i < msg.length; ++i) {
    out.data[i] = msg.data[i];
  }

  // wait for socket to be ready for writing
  if (waitForCanSock_) {
    std::unique_lock<std::mutex> lock(can_mutex_);
    while (doRun_ && canSock_ < 0) {
      can_condtion_.wait(lock);
    }
  }

  if (!doRun_) {
    throw std::runtime_error("shutdown");
  }

  // send msg
  {
    const auto res = ::write(canSock_, &out, sizeof(out));
    if (res < 0) {
      throw std::runtime_error("write() failed with: " + std::string(strerror(errno)));
    }
    if (res != sizeof(out)) {
      // re-open socket
      ::shutdown(canSock_, SHUT_RDWR);
      throw std::logic_error("write() buffer overflow!");
    }
  }
}

void ElmoMotorModule::setMotorModulo()
{
  const int32_t ticks_per_rev = motor_.enc_ticks_per_rev * motor_.gear_ratio;
  canopen_set_int('X', 'M', 1, -1 * ticks_per_rev);
  canopen_set_int('X', 'M', 2, ticks_per_rev);

  canSync();
}

void ElmoMotorModule::enableVelMode()
{
  // switch Unit Mode
  canopen_set_int('U', 'M', 0, 2);

  // set profile mode (only if Unit Mode = 2)
  canopen_set_int('P', 'M', 0, 1);

  // set maximum acceleration to X Incr/s^2
  canopen_set_int('A', 'C', 0, motor_.max_accel_enc_s);

  // set maximum decceleration to X Incr/s^2
  canopen_set_int('D', 'C', 0, motor_.max_accel_enc_s);

  canSync();
}

void ElmoMotorModule::setClock(rclcpp::Clock & clock)
{
  clock_ = clock;
}

void ElmoMotorModule::setHomeVel(double & homeVel)
{
  homeVel_ = homeVel;
}

void ElmoMotorModule::beginMotion()
{
  canOpenQuery('B', 'G', 0);
  canSync();
}

void ElmoMotorModule::stopMotion()
{
  canOpenQuery('S', 'T', 0);
  isStopped_ = true;
  canSync();
}

void ElmoMotorModule::startHoming()
{
  if (isHoming_ && !isOperational() && !isStopped()) {
    return;
  }
  stopMotion();
  // disableWatchdog();

  canopen_set_int('H', 'M', 1, 0);
  canSync();

  // configure homing sequences
  // setting the value such that increment counter resets after the homing event occurs
  canopen_set_int('H', 'M', 2, motor_.enc_home_offset);
  canSync();

  /** choosing channel/switch on which controller has to 
   listen for change of homing event(high/low/falling/rising) **/
  canopen_set_int('H', 'M', 3, HomeDigIn_);
  canSync();

  // choose the action that the controller shall perform after the homing event occurred
  // HM[4] = 0 : after Event stop immediately
  // HM[4] = 2 : do nothing
  canopen_set_int('H', 'M', 4, 2);
  canSync();

  // choose the setting of the position counter
  // (i.e. to the value defined in 2.a) after the homing event occured
  // HM[5] = 0 : absolute setting of position counter: PX = HM[2]
  canopen_set_int('H', 'M', 5, 0);
  canSync();

  motorSetVel(homeVel_);
  canSync();

  beginMotion();

  armHoming();

  canSync();

  isHomed_ = false;
  isHoming_ = true;
}

void ElmoMotorModule::armHoming()
{
  motor_.homing_state = -1;    // reset state
  motor_.homing_start_time = rclcpp::Clock().now();

  canopen_set_int('H', 'M', 1, 1);   // arm homeing
}

void ElmoMotorModule::motorSetVel(double & rotVelRadS)
{
  const double motor_vel_rev_s = motor_.gear_ratio * rotVelRadS / (2 * M_PI);
  const int32_t motor_vel_inc_s = motor_.rot_sign *
    static_cast<int>(motor_vel_rev_s * motor_.enc_ticks_per_rev);
  const int32_t lim_motor_vel_inc_s = std::min(
    std::max(
      motor_vel_inc_s,
      -motor_.max_vel_enc_s),
    motor_.max_vel_enc_s);

  canopen_set_int('J', 'V', 0, lim_motor_vel_inc_s);
}

void ElmoMotorModule::setHomeDigIn(int & HomeDigIn)
{
  HomeDigIn_ = HomeDigIn;
}

void ElmoMotorModule::finishHoming()
{
  stopMotion();

  // configureWatchdog();

  isHomed_ = true;
  isHoming_ = false;
  isMotorReset_ = true;

  // LastTrajectoryTime
}

bool ElmoMotorModule::isHoming()
{
  if (isHoming_) {
    canOpenQuery('H', 'M', 1);  
  }
  return isHoming_;
}

bool ElmoMotorModule::isHomed()
{
  const rclcpp::Time now = rclcpp::Clock().now();

  // Check to stop motor
  if (motor_.homing_state == 1) {
    stopMotion();
  }

  // Check for restart
  if (motor_.homing_state == -2) {
    armHoming();
  }

  // Check for timeout
  if ((now - motor_.homing_start_time).seconds() > 20) {
    std::cout<<"Homing timeout"<<std::endl; // later ROS log
    armHoming();
  }

  if (motor_.homing_state != 2) {
    isHomed_ = false;
  } else {
    isHomed_ = true;
  }

  return isHomed_;
}

bool ElmoMotorModule::isReset()
{
  return isMotorReset_;
}

bool ElmoMotorModule::isStopped()
{
  if (motor_.curr_enc_vel_inc_s > 100) {
    isStopped_ = false;
  } else {
    isStopped_ = true;
  }
  // Log
  return isStopped_;
}

bool ElmoMotorModule::isOperational()
{
  if (motor_.state == ST_OPERATION_ENABLED) {
    isMotorReset_ = false;
    return true;
  }
  return false;
}

void ElmoMotorModule::resetPosCounter()
{
  canopen_set_int('P','X', 0, 0);
  canSync();
}

void ElmoMotorModule::motorOn()
{
  canopen_set_int('M','O', 0, 1);
  canSync();
}

void ElmoMotorModule::motorOff()
{
  canopen_set_int('M','O', 0, 0);
  canSync();
}

void ElmoMotorModule::requestMotorStatus()
{
  canOpenQuery('S', 'R', 0);
  canSync();
}

void ElmoMotorModule::checkMotorTimeout(const double & timeout)
{
  auto now = rclcpp::Clock().now();
  if (motor_.status_recv_time < motor_.request_send_time &&
    ((now - motor_.request_send_time).nanoseconds() * 1.0 / 1000000000.0) > timeout)
  {
    motor_.state = ST_MOTOR_FAILURE;
  }
}

void ElmoMotorModule::canopen_set_int(
  char cmd_char_1,
  char cmd_char_2,
  int32_t index,
  int32_t data)
{
  can_msg_t msg;
  msg.id = motor_.node_id;
  msg.length = 8;
  msg.data[0] = cmd_char_1;
  msg.data[1] = cmd_char_2;
  msg.data[2] = index;
  // The two MSB (below) must be 0. Cf. DSP 301 Implementation guide p. 39.
  msg.data[3] = (index >> 8) & 0x3F;
  msg.data[4] = data;
  msg.data[5] = data >> 8;
  msg.data[6] = data >> 16;
  msg.data[7] = data >> 24;
  transmitCan(msg);
}

void ElmoMotorModule::canOpenQuery(
  char cmd_char_1,
  char cmd_char_2,
  int32_t index)
{
  can_msg_t msg;
  msg.id = motor_.node_id;
  msg.length = 8;
  msg.data[0] = cmd_char_1;
  msg.data[1] = cmd_char_2;
  msg.data[2] = index;
  // The two MSB (below) must be 0. Cf. DSP 301 Implementation guide p. 39.
  msg.data[3] = (index >> 8) & 0x3F;
  transmitCan(msg);
}

void ElmoMotorModule::canSync()
{
  ::usleep(10000);    // workaround, sleep for around 10 msgs
}
