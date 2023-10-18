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

ElmoMotorModule::ElmoMotorModule(motor_t motor, rclcpp::Logger logger)
  :motor_(motor), logger_(logger)
{
  // configuring CAN for the motor transmission
  motor_.can_Tx_PDO1 = motor_.node_id + 0x180;
  motor_.can_Tx_PDO2 = motor_.node_id + 0x280;
  motor_.can_Rx_PDO2 = motor_.node_id + 0x300;
  motor_.can_Tx_SDO = motor_.node_id + 0x580;
  motor_.can_Rx_SDO = motor_.node_id + 0x600;
  last_update_time_ = clock_.now();
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
    while (canSock_ < 0) {
      can_condtion_.wait(lock);
    }
  }

  if (shutdown_) {
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

void ElmoMotorModule::shutdown()
{
  {
    // std::lock_guard<std::mutex> lock(m_node_mutex);

    // disable waiting for CAN socket, since we are shutting down
    waitForCanSock_ = false;
    shutdown_ = true;
  }
  {
    // std::lock_guard<std::mutex> lock(m_can_mutex);
    if (canSock_ >= 0) {
      ::close(canSock_);
      canSock_ = -1;
    }
  }
}

void ElmoMotorModule::setCanIface(std::string & canIface)
{
  canIface_ = canIface;
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
  if (isHoming_ ) {
    return;
  }
  stopMotion();
  disableWatchdog();

  canopen_set_int('H', 'M', 1, 0);
  canSync();

  // configure homing sequences
  // setting the value such that increment counter resets after the homing event occurs
  canopen_set_int('H', 'M', 2, -motor_.enc_home_offset);
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
  motor_.homing_start_time = clock_.now();
  RCLCPP_INFO(logger_, "arming");

  canopen_set_int('H', 'M', 1, 1);   // arm homeing
}

void ElmoMotorModule::motorSetVel(double & linVelms)
{
  const double motor_vel_rev_s = motor_.gear_ratio * linVelms / motor_.meters_per_revolution;
  const int32_t motor_vel_inc_s = static_cast<int>(motor_vel_rev_s * motor_.enc_ticks_per_rev);

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

void ElmoMotorModule::setCommonMotorParams(
  double & control_rate,
  int & heartbeat_divider)
{
  control_rate_ = control_rate;
  heartbeat_divider_ = heartbeat_divider;
}

void ElmoMotorModule::sendHeartBeat()
{
  can_msg_t msg;        // send heartbeat message
  msg.id = 0x700;
  msg.length = 5;
  transmitCan(msg);
}

void ElmoMotorModule::enableWatchdog()
{
  // configure to fail after missing 3 heartbeats
  const int heartbeat_time_ms = 4 * 1000 * heartbeat_divider_ / control_rate_;
  const int pc_node_id = 0x00;

  // consumer (PC) heartbeat time
  canOpenSDODownload(0x1016, 1, (pc_node_id << 16) | heartbeat_time_ms);

  // error behavior after failure: 0=pre-operational, 1=no state change, 2=stopped"
  canOpenSDODownload(0x1029, 1, 2);

  canSync();

  // motor behavior after heartbeat failure: "quick stop"
  canOpenSDODownload(0x6007, 0, 3);

  // activate emergency events: "heartbeat event"
  // Object 0x2F21 = "Emergency Events" which cause an Emergency Message
  // Bit 3 is responsible for Heartbeart-Failure.--> Hex 0x08
  canOpenSDODownload(0x2F21, 0, 0x08);

  canSync();
}

void ElmoMotorModule::disableWatchdog()
{
  // Motor action after Hearbeat-Error: No Action
  canOpenSDODownload(0x6007, 0, 0);

  // Error Behavior: No state change
  canOpenSDODownload(0x1029, 1, 1);

  // Deacivate emergency events: "heartbeat event"
  // Object 0x2F21 = "Emergency Events" which cause an Emergency Message
  // Bit 3 is responsible for Heartbeart-Failure.
  canOpenSDODownload(0x2F21, 0, 0x00);

  canSync();
}

void ElmoMotorModule::finishHoming()
{
  stopMotion();

  enableWatchdog();

  isHomed_ = true;
  isHoming_ = false;
  isMotorReset_ = true;
}

bool ElmoMotorModule::isHoming()
{
  return isHoming_;
}

void ElmoMotorModule::reqHomStatus()
{
  canOpenQuery('H', 'M', 1);
}

bool ElmoMotorModule::isHomed()
{
  return isHomed_;
}

bool ElmoMotorModule::checkHomingDone()
{
  const rclcpp::Time now = clock_.now();

  if (motor_.homing_state == 1) {
    stopMotion();
    motor_.homing_state = 2;
  }

  // Check for restart
  if (motor_.homing_state == -2) {
    armHoming();
  }

  // Check for timeout
  if ((now - motor_.homing_start_time).seconds() > 20) {
    RCLCPP_ERROR(logger_, "Homing timeout");
    armHoming();
  }

  if (motor_.homing_state != 2) {
    isHomed_ = false;
  } else {
    RCLCPP_INFO(logger_, "Homing Finishing");
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

void ElmoMotorModule::setDigitalInput()
{
  canopen_set_int('I', 'L', 11, 7);
}

void ElmoMotorModule::requestMotorStatus()
{
  canOpenQuery('S', 'R', 0);
  canSync();
}

void ElmoMotorModule::sendSyncMsg()
{
  can_msg_t msg;
  msg.id = 0x80;
  msg.length = 0;
  transmitCan(msg);
}

void ElmoMotorModule::checkMotorTimeout(const double & timeout)
{
  auto now = clock_.now();
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
  msg.id = motor_.can_Rx_PDO2;
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
  msg.id = motor_.can_Rx_PDO2;
  msg.length = 4;
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

bool ElmoMotorModule::isSocketOpened()
{
  return sockStatus_;
}

void ElmoMotorModule::openSocket()
{
  if (!isError_ || canSock_ < 0) {
    if (canSock_ >= 0) {
      ::close(canSock_); // close first
    }
    try {
      // Open Socket
      canSock_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
      if (canSock_ < 0) {
        throw std::runtime_error("socket() failed!");
        RCLCPP_ERROR(logger_, "Socket() failed!");
      }
      ::ifreq ifr = {};
      ::strncpy(ifr.ifr_name, canIface_.c_str(), IFNAMSIZ);

      if (::ioctl(canSock_, SIOCGIFINDEX, &ifr) < 0) {
        throw std::runtime_error("ioctl() failed!");
      }

      // bind to interface
      ::sockaddr_can addr = {};
      addr.can_family = AF_CAN;
      addr.can_ifindex = ifr.ifr_ifindex;
      
      if (::bind(canSock_, (::sockaddr *)(&addr), sizeof(addr)) < 0) {
        throw std::runtime_error("bind() failed!");
      }

      RCLCPP_INFO_STREAM(
        logger_, "CAN interface '" << canIface_ << "' opened successfully.");
      sockStatus_ = true;
      isError_ = false;
    } catch (const std::exception & ex) {
      RCLCPP_WARN_STREAM(
        logger_, "Failed to open CAN interface '" << canIface_ << "': "
                                                             << ex.what() << " (" <<
          ::strerror(errno) << ")");
      isError_ = true;
      return;
    }
    can_condtion_.notify_all();  // notify that socket is ready
  }
}

void ElmoMotorModule::checkSocketError(bool & isError)
{
  isError = isError_;
}

void ElmoMotorModule::readIOs()
{
  canOpenQuery('I', 'B', 10);
  canOpenQuery('I', 'B', 11);
}

void ElmoMotorModule::readMotor()
{
  
  // close socket if error
  if (isError_) {
    ::close(canSock_);
    sockStatus_ = false;
    ::usleep(1000 * 1000);  // in case of error sleep some time
    canSock_ = -1;
    can_condtion_.notify_all();
    return;
  }
  // read a frame
  ::can_frame frame = {};
  const auto res = ::read(canSock_, &frame, sizeof(frame));

  if (res != sizeof(frame)) {
    RCLCPP_WARN_STREAM(logger_, "read() failed with " << ::strerror(errno));
    return;
  }

  // convert frame
  can_msg_t msg;
  msg.id = frame.can_id & 0x1FFFFFFF;
  msg.length = frame.can_dlc;
  for (int i = 0; i < frame.can_dlc; ++i) {
    msg.data[i] = frame.data[i];
  }
  // process it
  {
    waitForCanSock_ = false;    // disable waiting for transmit (avoid dead-lock)
    std::lock_guard<std::mutex> lock(node_mutex_);
    try {
      handle(msg);
    } catch (const std::exception & ex) {
      RCLCPP_WARN_STREAM(logger_, ex.what());
    }
    waitForCanSock_ = true;     // enable waiting again
  }

  // close socket
  // {
  //   // std::lock_guard<std::mutex> lock(m_can_mutex);
  //   if (canSock_ >= 0) {
  //     ::close(canSock_);
  //     canSock_ = -1;
  //   }
  //   can_condtion_.notify_all();     // notify that socket is closed
  // }
}

void ElmoMotorModule::handle(const can_msg_t & msg)
{
  if (msg.id == motor_.can_Tx_PDO1) {
    handlePDO1(msg);
  }
  if (msg.id == motor_.can_Tx_PDO2) {
    handlePDO2(msg);
  }

  if ( (motor_.update_recv_time > last_sync_time_) ) {
    curr_vel_ = currVel();
    auto pose = curr_vel_ *(clock_.now() - last_update_time_).seconds();
    last_update_time_ = clock_.now();
    curr_pos_ += pose;
    motor_.prev_enc_pos_inc = motor_.curr_enc_pos_inc;
  }
  
}

void ElmoMotorModule::setSyncTime(rclcpp::Time last_sync_time)
{
  last_sync_time_ = last_sync_time;
}

double ElmoMotorModule::getPose()
{
  return curr_pos_;
}

double ElmoMotorModule::getVel()
{
  return curr_vel_;
}

void ElmoMotorModule::digInStates(int32_t & dig_1, int32_t & dig_2)
{
  dig_1 = dig_ins_[0];
  dig_2 = dig_ins_[1];
}

void ElmoMotorModule::handlePDO1(const can_msg_t & msg)
{
  motor_.curr_enc_pos_inc = read_int32(msg, 0);
  motor_.curr_enc_vel_inc_s = read_int32(msg, 4);
  motor_.update_recv_time = clock_.now();
}

void ElmoMotorModule::handlePDO2(const can_msg_t & msg)
{
  if (msg.data[0] == 'S' && msg.data[1] == 'R') {
    const auto prev_status = motor_.curr_status;
    motor_.curr_status = read_int32(msg, 4);
    evalStatus(prev_status);
    motor_.status_recv_time = clock_.now();
  }
  if (msg.data[0] == 'M' && msg.data[1] == 'F') {
    const auto prev_status = motor_.curr_motor_failure;
    motor_.curr_motor_failure = read_int32(msg, 4);
    evaluateFailure(prev_status);
  }
  if (msg.data[0] == 'H' && msg.data[1] == 'M') {
    if (msg.data[4] == 0) {      // check if motor says homing finished

      if (motor_.homing_state == 0) {
        if (((clock_.now() - motor_.homing_start_time).nanoseconds() * 1.0 /
          1000000000.0) > 0.5)
        {
          motor_.homing_state = 1;   // only go to finish after active for some time
        } else {
          motor_.homing_state = -2;  // restart, since it finished too soon
        }
      } else if (motor_.homing_state == -1) {
        motor_.homing_state = -2;    // restart, since it was never active
      }
    } else {
      motor_.homing_state = 0;       // homing active
    }
  }
  if (msg.data[0] == 'I' && msg.data[1] == 'B') {
    auto data = read_int32(msg, 4);
    if (msg.data[2] == 10) {
      dig_ins_[0] = data; 
    } else if (msg.data[2] == 11) {
      dig_ins_[1] = data;
    }
  }
}

double ElmoMotorModule::currPose()
{
  if (motor_.curr_enc_pos_inc < 0) {
    motor_.curr_enc_pos_inc = 30000 - abs(motor_.curr_enc_pos_inc);
  }

  return static_cast<double>(motor_.meters_per_revolution * std::abs(motor_.curr_enc_pos_inc - motor_.prev_enc_pos_inc)) /
         motor_.enc_ticks_per_rev / motor_.gear_ratio;
}

double ElmoMotorModule::currVel()
{
  return static_cast<double>(motor_.meters_per_revolution * motor_.curr_enc_vel_inc_s) /
         motor_.enc_ticks_per_rev / motor_.gear_ratio;
}

int32_t ElmoMotorModule::read_int32(const can_msg_t & msg, int offset) const
{
  if (offset < 0 || offset > 4) {
    throw std::logic_error("invalid offset");
  }
  int32_t value = 0;
  ::memcpy(&value, msg.data + offset, 4);
  return value;
}

float ElmoMotorModule::read_float(const can_msg_t & msg, int offset) const
{
  if (offset < 0 || offset > 4) {
    throw std::logic_error("invalid offset");
  }
  float value = 0;
  ::memcpy(&value, msg.data + offset, 4);
  return value;
}

void ElmoMotorModule::evalStatus(int32_t prev_status)
{
  if (motor_.curr_status & 1) {
    if (motor_.curr_status != prev_status) {
      if ((motor_.curr_status & 0xE) == 2) {
        RCLCPP_ERROR_STREAM(
          logger_, motor_.joint_name << ": drive error under voltage");
      } else if ((motor_.curr_status & 0xE) == 4) {
        RCLCPP_ERROR_STREAM(
          logger_, motor_.joint_name << ": drive error over voltage");
      } else if ((motor_.curr_status & 0xE) == 10) {
        RCLCPP_ERROR_STREAM(
          logger_, motor_.joint_name << ": drive error short circuit");
      } else if ((motor_.curr_status & 0xE) == 12) {
        RCLCPP_ERROR_STREAM(
          logger_, motor_.joint_name << ": drive error over-heating");
      } else {
        RCLCPP_ERROR_STREAM(
          logger_,
          motor_.joint_name << ": unknown failure: " << (motor_.curr_status & 0xE));
      }
    }

    // request detailed description of failure
    canOpenQuery('M', 'F', 0);

    motor_.state = ST_MOTOR_FAILURE;
  } else if (motor_.curr_status & (1 << 6)) {
    // general failure
    if (motor_.curr_status != prev_status) {
      RCLCPP_ERROR_STREAM(logger_, motor_.joint_name << ": failure latched");
    }

    // request detailed description of failure
    canOpenQuery('M', 'F', 0);

    motor_.state = ST_MOTOR_FAILURE;
  } else {
    // check if Bit 4 (-> Motor is ON) ist set
    if (motor_.curr_status & (1 << 4)) {
      if (motor_.state != ST_OPERATION_ENABLED) {
        RCLCPP_INFO_STREAM(logger_, motor_.joint_name << ": operation enabled");
      }
      motor_.state = ST_OPERATION_ENABLED;
    } else {
      if (motor_.state != ST_OPERATION_DISABLED) {
        RCLCPP_WARN_STREAM(logger_, motor_.joint_name << ": operation disabled");
      }
      motor_.state = ST_OPERATION_DISABLED;
    }
  }
}

void ElmoMotorModule::evaluateFailure(int32_t prev_status)
{
  if (motor_.curr_motor_failure != prev_status) {
    if (motor_.curr_motor_failure & (1 << 2)) {
      RCLCPP_ERROR_STREAM(
        logger_,
        motor_.joint_name << ": motor failure: feedback loss");
    } else if (motor_.curr_motor_failure & (1 << 3)) {
      RCLCPP_ERROR_STREAM(
        logger_, motor_.joint_name << ": motor failure: peak current exceeded");
    } else if (motor_.curr_motor_failure & (1 << 7)) {
      RCLCPP_ERROR_STREAM(
        logger_, motor_.joint_name << ": motor failure: speed track error");
    } else if (motor_.curr_motor_failure & (1 << 8)) {
      RCLCPP_ERROR_STREAM(
        logger_, motor_.joint_name << ": motor failure: position track error");
    } else if (motor_.curr_motor_failure & (1 << 17)) {
      RCLCPP_ERROR_STREAM(
        logger_, motor_.joint_name << ": motor failure: speed limit exceeded");
    } else if (motor_.curr_motor_failure & (1 << 21)) {
      RCLCPP_ERROR_STREAM(
        logger_, motor_.joint_name << ": motor failure: motor stuck");
    } else {
      RCLCPP_ERROR_STREAM(
        logger_, motor_.joint_name << ": motor failure: " << motor_.curr_motor_failure);
    }
  }
}
