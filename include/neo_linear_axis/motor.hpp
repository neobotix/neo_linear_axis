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

#ifndef NEO_LINEAR_AXIS__MOTOR_HPP_
#define NEO_LINEAR_AXIS__MOTOR_HPP_

#include "rclcpp/rclcpp.hpp"

enum motor_state_e
{
  ST_PRE_INITIALIZED,
  ST_OPERATION_ENABLED,
  ST_OPERATION_DISABLED,
  ST_MOTOR_FAILURE
};

struct motor_t
{
  std::string joint_name;         // ROS joint name
  int32_t node_id = 2;          // motor "CAN ID"
  int32_t rot_sign = 0;         // motor rotation direction
  int32_t enc_ticks_per_rev = 10000;      // encoder ticks per motor revolution
  int32_t enc_home_offset = 0;      // encoder offset for true home position
  int32_t max_vel_enc_s = 200000;     // max motor velocity in ticks/s (positive)
  int32_t max_accel_enc_s = 500000;    // max motor acceleration in ticks/s^2 (positive)
  int32_t meters_per_revolution = 10;
  int32_t can_Tx_PDO1 = -1;
  int32_t can_Tx_PDO2 = -1;
  int32_t can_Rx_PDO2 = -1;
  int32_t can_Tx_SDO = -1;
  int32_t can_Rx_SDO = -1;
  int32_t gear_ratio = 3;         // gear ratio
  double torque_constant = 0;       // conversion factor from current to torque

  motor_state_e state = ST_PRE_INITIALIZED;
  int32_t curr_enc_pos_inc = 0;     // current encoder position value in ticks
  int32_t curr_enc_vel_inc_s = 0;     // current encoder velocity value in ticks/s
  int32_t curr_status = 0;        // current status as received by SR msg
  int32_t curr_motor_failure = 0;     // current motor failure status as received by MF msg
  double curr_torque = 0;         // current measure motor torque
  rclcpp::Time request_send_time;     // time of last status update request
  rclcpp::Time status_recv_time;        // time of last status update received
  rclcpp::Time update_recv_time;        // time of last sync update received
  rclcpp::Time homing_start_time;     // time of homing start
  // current homing state (-2 = restart, -1 = unknown, 0 = active, 1 = finished, 2 = done)
  int homing_state = -1;
};

#endif // NEO_LINEAR_AXIS__MOTOR_HPP_
