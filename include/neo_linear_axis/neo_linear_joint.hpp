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
#include <mutex>

#ifndef NEO_LINEAR_AXIS__NEO_LINEAR_JOINT_HPP_
#define NEO_LINEAR_AXIS__NEO_LINEAR_JOINT_HPP_

class NeoLinearJoint
{
public:
  // Defining constructor, later will be updated with args
  NeoLinearJoint(motor_t motor);
  ~NeoLinearJoint();

  void initialize();

  // void sdoDownload(
  //   int & node_id, int32_t obj_index, int32_t obj_sub_index,
  //   int32_t data);
  // void transmitCan(const can_msg_t & msg);
  // void canSync();
  // int32_t read_int32(const can_msg_t & msg, int offset);
  // float read_float(const can_msg_t & msg, int offset)
private:
  std::mutex m_node_mutex;
  bool waitForCanSock = false;
  motor_t motor_;
};

#endif // NEO_LINEAR_AXIS__NEO_LINEAR_JOINT_HPP_