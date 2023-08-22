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
}

NeoLinearJoint::~NeoLinearJoint()
{}

void NeoLinearJoint::initialize()
{
  std::lock_guard<std::mutex> lock(node_mutex_);

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

  // reset motor states
}

void NeoLinearJoint::transmitCan(const can_msg_t & msg)
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

void NeoLinearJoint::canSync()
{
  ::usleep(10000);    // workaround, sleep for around 10 msgs
}
