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

ElmoMotorModule::ElmoMotorModule(motor_t motor)
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
