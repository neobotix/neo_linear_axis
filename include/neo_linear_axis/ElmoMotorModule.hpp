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

#include <neo_linear_axis/can_msg.hpp>
#include <neo_linear_axis/motor.hpp>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

#ifndef NEO_LINEAR_AXIS__ELMO_MOTOR_MODULE_HPP_
#define NEO_LINEAR_AXIS__ELMO_MOTOR_MODULE_HPP_

class ElmoMotorModule
{
public:
  ElmoMotorModule(motor_t & motor);
  ~ElmoMotorModule();

  void initializeNmt();
  void canSync();
  void transmitCan(const can_msg_t & msg);
  void setMotorModulo();
  void configurePDOMapping();
  void resetPosCounter();
  void motorOn();
  void motorOff();
  void requestMotorStatus();
  void checkMotorTimeout(const double & timeout);
  void motorSetVel(double & linVelms);
  void setClock(rclcpp::Clock & clock);
  bool isReset();
  bool isOperational();
  void enableVelMode();
  bool isHomed();
  void setHomeVel(double & homeVel);
  void startHoming();
  void beginMotion();
  void stopMotion();
  bool isStopped();
  void setHomeDigIn(int & HomeDigIn);
  bool isHoming();
  void finishHoming();
  void setCommonMotorParams(
    double & control_rate,
    int & heartbeat_divider);
  void sendSyncMsg();
  void sendHeartBeat();
  bool isSocketOpened();
  void openSocket();
  void setCanIface(std::string & canIface);
  void readMotor();
  void shutdown();
  double getPose();
  double getVel();
  void setSyncTime(rclcpp::Time last_sync_time);

private:
  void canopen_set_int(char cmd_char_1, char cmd_char_2, int32_t index, int32_t data);
  void canOpenSDODownload(int32_t obj_index, int32_t obj_sub_index, int32_t data);
  void canOpenQuery(char cmd_char_1, char cmd_char_2, int32_t index);
  void armHoming();
  void disableWatchdog();
  void enableWatchdog();
  void handle(const can_msg_t & msg);
  void handlePDO1(const can_msg_t & msg);
  void handlePDO2(const can_msg_t & msg);
  void evalStatus(int32_t prev_status);
  void evaluateFailure(int32_t prev_status);
  int32_t read_int32(const can_msg_t & msg, int offset) const;
  float read_float(const can_msg_t & msg, int offset) const;
  double currPose();
  double currVel();


  motor_t motor_;
  bool waitForCanSock_ = false;
  std::mutex node_mutex_;
  std::mutex can_mutex_;
  rclcpp::Clock clock_;
  bool isMotorReset_ = true;
  bool isHomed_ = false;
  bool isHoming_ = false;
  bool isStopped_ = true;
  bool sockStatus_ = false;
  int HomeDigIn_;
  rclcpp::Time last_sync_time_;

  volatile bool doRun_ = true;

  std::condition_variable can_condtion_;
  int canSock_ = -1;

  double homeVel_;
  double curr_pos_ = 0.0;
  double curr_vel_ = 0.0;
  int heartbeat_divider_;
  double control_rate_;

  bool isError_ = false;
  std::string canIface_;
  bool shutdown_=false;
};
#endif // NEO_LINEAR_AXIS__ELMO_MOTOR_MODULE_HPP_
