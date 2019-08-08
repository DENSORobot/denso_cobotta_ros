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
#ifndef GRIPPER_BASE_H_
#define GRIPPER_BASE_H_

// C++ Standards
#include <iostream>
#include <functional>
#include <mutex>
#include <memory>
#include <string>

// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <std_msgs/Bool.h>

// COBOTTA
#include <sys/ioctl.h>
#include <fcntl.h>
#include "denso_cobotta_lib/cobotta_ioctl.h"
#include "denso_cobotta_lib/driver.h"
#include "denso_cobotta_lib/cobotta_exception.h"
#include "denso_cobotta_driver/RobotState.h"
#include "denso_cobotta_lib/cobotta_common.h"
#include "denso_cobotta_lib/gripper.h"

namespace denso_cobotta_gripper
{

class GripperBase
{
public:
  static constexpr const char* TAG = "GripperBase";

  GripperBase();
  virtual ~GripperBase() = default;
  virtual bool read(void) = 0;
  virtual bool write(void) = 0;
  virtual bool update(void) = 0;
  virtual bool initialize(ros::NodeHandle& nh);

  void checkMotorState(void);
  bool isMotorOn(void) const;
  bool isGraspState() const;
  int getFd() const;

  static void sendStayHere(int fd);

protected:
  bool openDeviceFile(void);
  bool verifyGripperType(const std::string& gripper_str);
  bool loadConfigParams(ros::NodeHandle& nh);
  bool initSubscribers(ros::NodeHandle& nh);

  virtual bool publish(void) = 0;
  virtual bool subscribe(void) = 0;

  // Subscriber callback functions.
  void subRobotState(const denso_cobotta_driver::RobotState& msg);
  void subGraspState(const std_msgs::Bool::ConstPtr& msg);

  bool initCurPos(void);
  bool isBusy(void);
  bool recvEncoderData(void);
  bool calcGripperCommand(void);
  bool stopMove(void);
  bool sendServoUpdateData(void);

  int fd_;
  enum denso_cobotta_lib::cobotta::GripperType gripper_type_;
  std::mutex move_lock_;
  bool motor_on_;
  bool moving_;
  bool grasp_state_;

  void setMotorOn(bool);

  // Control parameters.
  double current_cmd_position_;
  double current_cmd_velocity_;
  double current_target_position_;
  double current_speed_percentage_;
  double current_effort_;
  double start_position_;

  double current_position_;
  double max_soft_limit_;
  double min_soft_limit_;
  double max_velocity_;
  double max_acceleration_;
  double max_speed_percentage_;
  double min_speed_percentage_;
  double max_effort_;
  double min_effort_;
  double coeff_outpos_to_pulse_;
  double coeff_effort_to_torque_;
  constexpr static int TIMES_CURRENT = 1000;

  // Subscribers.
  ros::Subscriber sub_robot_state_;
  ros::Subscriber sub_grasp_state_;
};

}  // namespace denso_cobotta_gripper
#endif
