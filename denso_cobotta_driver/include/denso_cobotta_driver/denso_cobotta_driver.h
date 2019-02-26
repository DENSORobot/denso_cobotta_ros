/*
 * Copyright (C) 2018-2019  DENSO WAVE INCORPORATED
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

#ifndef _DENSO_COBOTTA_DRIVER_H_
#define _DENSO_COBOTTA_DRIVER_H_

// C++ standard
#include <iostream>
#include <string>
#include <cerrno>
#include <cstring>

// ROS
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include "denso_cobotta_driver/GetBrakeState.h"
#include "denso_cobotta_driver/GetMotorState.h"
#include "denso_cobotta_driver/SetBrakeState.h"
#include "denso_cobotta_driver/SetMotorState.h"
#include "denso_cobotta_driver/SetLEDState.h"
#include "denso_cobotta_driver/ClearError.h"
#include "denso_cobotta_driver/ClearRobotError.h"
#include "denso_cobotta_driver/ClearSafeState.h"
#include "denso_cobotta_driver/RobotState.h"
#include "denso_cobotta_driver/SafeState.h"

// COBOTTA device driver
#include <fcntl.h>
#include <sys/ioctl.h>
#include "denso_cobotta_driver/cobotta_ioctl.h"


namespace denso_cobotta_driver
{
class DensoCobottaDriver
{
public:

  enum LedColor : uint64_t
  {
    no_change = 0,
    white = 0x00645028,
    red = 0x00640000,
    green = 0xff053205,
    blue = 0x00000032,
    yellow = 0x005f3c00
  };

  DensoCobottaDriver();
  virtual ~DensoCobottaDriver();

  bool initialize(ros::NodeHandle& nh);
  void terminate();
  bool checkState();
  void publishState();
  bool fetchMiniInput();

  // Service callback functions.
  bool setMotorStateCB(SetMotorState::Request& req, SetMotorState::Response& res);
  bool getMotorStateCB(GetMotorState::Request& req, GetMotorState::Response& res);
  bool setBrakeStateCB(SetBrakeState::Request& req, SetBrakeState::Response& res);
  bool getBrakeStateCB(GetBrakeState::Request& req, GetBrakeState::Response& res);
  bool clearErrorCB(ClearError::Request& req, ClearError::Response& res);
  bool clearRobotErrorCB(ClearError::Request& req, ClearError::Response& res);
  bool clearSafeStateCB(ClearError::Request& req, ClearError::Response& res);
  bool setLEDStateCB(SetLEDState::Request& req, SetLEDState::Response& res);

  // Subscriber callback functions.
  void subMiniIOOutputCB(const std_msgs::UInt16::ConstPtr& msg);

private:
  enum class MotorState
  {
    MotorOFF,
    MotorON
  };
  enum class SafeStateType
  {
    Error,
    Standby,
    Normal
  };

  struct LinuxDriverVersion
  {
    uint8_t major;
    uint8_t minor;
    uint8_t rev;
    uint8_t build;
  };

  const static uint16_t ACYCLIC_COMM_ADDRESS[2];
  const static uint16_t ACYCLIC_COMM_VALUE[2][9];

  bool getDriverVersion(struct LinuxDriverVersion* version);
  bool motorON();
  bool motorOFF();
  bool clearError();
  bool clearRobotError();
  bool clearSafeState();
  bool motorState(long* state);
  bool setBrake();
  bool getBrake();
  bool setLED(const uint8_t red, const uint8_t green, const uint8_t blue, const uint8_t blink_rate);
  bool setLED(const uint64_t value);

  // Service server
  ros::ServiceServer sv_set_motor_;
  ros::ServiceServer sv_get_motor_;
  ros::ServiceServer sv_clear_error_;
  ros::ServiceServer sv_clear_robot_error_;
  ros::ServiceServer sv_clear_safe_state_;
  ros::ServiceServer sv_set_brake_;
  ros::ServiceServer sv_get_brake_;
  ros::ServiceServer sv_set_LED_;

  // Publisher
  ros::Publisher pub_position_button_;
  ros::Publisher pub_open_button_;
  ros::Publisher pub_close_button_;
  ros::Publisher pub_miniIO_input_;
  ros::Publisher pub_robot_state_;
  ros::Publisher pub_safe_state_;

  // Subscriber
  ros::Subscriber sub_miniIO_output_;

  int fd_;
  bool level5_error_flag;
  bool safe_state_error_flag;
  bool robot_error_flag;

  std_msgs::Bool position_button_state_;
  std_msgs::Bool open_button_state_;
  std_msgs::Bool close_button_state_;

  IOCTL_DATA_BRAKE brake_state_;
};
}  // namespace denso_cobotta_driver

#endif // _DENSO_COBOTTA_DRIVER_H_
