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

#ifndef _DENSO_COBOTTA_HW_H_
#define _DENSO_COBOTTA_HW_H_

// C++ standard
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <cerrno>

// ROS
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include "denso_cobotta_driver/SetMotorState.h"
#include "denso_cobotta_driver/SetLEDState.h"
#include "denso_cobotta_driver/ClearError.h"

// COBOTTA device driver
#include <fcntl.h>
#include <signal.h>
#include <sys/ioctl.h>
#include "denso_cobotta_driver/cobotta_ioctl.h"
#include "denso_cobotta_driver/cobotta_common.h"
#include "denso_cobotta_driver/RobotState.h"

namespace denso_cobotta_control
{
using namespace cobotta_common;

class DensoCobottaHW : public hardware_interface::RobotHW
{
public:
  DensoCobottaHW();
  virtual ~DensoCobottaHW();

  bool initialize(ros::NodeHandle& nh);

  bool Read(ros::Time, ros::Duration);
  bool Write(ros::Time, ros::Duration);
  bool isMotorOn();

  // Subscriber callback functions.
  void subRobotStateCB(const denso_cobotta_driver::RobotState& msg);

  // Subscriber
  ros::Subscriber sub_robot_state_;

private:
  // Constants
  const static double GEAR_RATIOS[CONTROL_JOINT_MAX];
  const static int DIRECTION[CONTROL_JOINT_MAX];

  void calcPulseConversionCoeff();
  bool setServoUpdateData();
  bool getEncoderData();

  hardware_interface::JointStateInterface jntStInterface_;
  hardware_interface::PositionJointInterface posJntInterface_;

  double cmd_[CONTROL_JOINT_MAX];
  double pos_[CONTROL_JOINT_MAX];
  double vel_[CONTROL_JOINT_MAX];
  double eff_[CONTROL_JOINT_MAX];
  double coeff_pulse_to_outpos_[CONTROL_JOINT_MAX];
  double coeff_outpos_to_pulse_[CONTROL_JOINT_MAX];
  bool motor_on_; // true:on false:off

  IOCTL_DATA_UPDATE upd_;

  int fd_;
};
}  // namespace denso_cobotta_control
#endif // _DENSO_COBOTTA_HW_H_
