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

#ifndef _DENSO_COBOTTA_GRIPPER_H_
#define _DENSO_COBOTTA_GRIPPER_H_

// C++ standards
#include <iostream>
#include <string>
#include <memory>
#include <mutex>

// ROS
#include <ros/ros.h>
#include <control_msgs/GripperCommandAction.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <actionlib/server/simple_action_server.h>
#include "denso_cobotta_gripper/GripperMoveAction.h"

// COBOTTA device driver
#include <sys/ioctl.h>
#include <fcntl.h>
#include "denso_cobotta_driver/cobotta_ioctl.h"
#include "denso_cobotta_driver/RobotState.h"

namespace denso_cobotta_gripper
{
class DensoCobottaGripper
{
public:
  DensoCobottaGripper();
  virtual ~DensoCobottaGripper();

  bool initialize(ros::NodeHandle& nh);
  bool gripperMove(const double& target_positiion, const double& speed, const double& effort);
  bool gripperStop();
  bool gripperCurPos(double& current_position);
  bool gripperBusyState(bool& state);

  bool Read();
  bool Write();
  bool isMotorOn();

private:
  // Constants
  constexpr static double MAX_POSITION = 0.03;
  constexpr static double MIN_POSITION = 0.0;
  constexpr static double MAX_VELOCITY = 0.08;
  constexpr static double MAX_ACCELERATION = 0.8;
  constexpr static double MAX_SPEED = 100.0;
  constexpr static double MIN_SPEED = 1.0;
  constexpr static double MAX_EFFORT = 20.0;
  constexpr static double MIN_EFFORT = 6.0;
  constexpr static double COEFF_OUTPOS_TO_PULSE = 2481230.769;
  constexpr static double COEFF_EFFORT_TO_TORQUE = 0.0305;

  bool setGripperCommand();
  bool setServoUpdateData();
  bool getEncoderData();

  // Subscriber callback functions.
  void subRobotStateCB(const denso_cobotta_driver::RobotState& msg);

  // Action callback functions.
  bool gripperCommandActionGoalCB(const control_msgs::GripperCommandGoalConstPtr& goal);
  bool gripperMoveActionCB(const GripperMoveGoalConstPtr& goal);
  void cancelCB();
  void actionFeedback();

  // Action server
  std::shared_ptr<actionlib::SimpleActionServer<GripperMoveAction> > as_gripper_move_;
  std::shared_ptr<actionlib::SimpleActionServer<control_msgs::GripperCommandAction> > as_gripper_cmd_;

  // Publisher
  ros::Publisher pub_joint_state_;

  // Subscriber
  ros::Subscriber sub_robot_state_;

  double current_position_;
  double current_cmd_velocity_;
  double current_cmd_position_;
  double current_target_position_;
  double current_speed_;
  double current_effort_;
  double start_position_;
  bool move_complete_;
  bool motor_on_; // true:on false:off

  int fd_;

  std::mutex gripper_mtx_;
};

}  // namespace denso_cobotta_gripper

#endif // _DENSO_COBOTTA_GRIPPER_H_
