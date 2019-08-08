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
#ifndef GRIPPER_PARALLEL_H_
#define GRIPPER_PARALLEL_H_

#include <iostream>
#include <string>
#include <memory>
#include <mutex>

#include <control_msgs/GripperCommandAction.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>

#include "denso_cobotta_gripper/GripperMoveAction.h"

#include "denso_cobotta_lib/driver.h"
#include "denso_cobotta_gripper/gripper_base.h"

namespace denso_cobotta_gripper
{
class GripperParallel : public GripperBase
{
public:
  static constexpr const char* TAG = "GripperParallel";

  GripperParallel();
  virtual ~GripperParallel() = default;
  bool initialize(ros::NodeHandle& nh) override;
  bool read(void) override;
  bool write(void) override;
  bool update(void) override;

  static void sendStayHere(int fd);

private:
  bool calcGripperCommand(void);
  bool sendServoUpdateData(void);

  bool publish(void);
  bool subscribe(void);
  bool initActionServers(ros::NodeHandle& nh);
  bool initGripperMove(const double& target_position, const double& speed_percentage, const double& effort);

  // Action callback functions.
  bool gripperCommandActionGoal(const control_msgs::GripperCommandGoalConstPtr& goal);
  bool gripperMoveActionGoal(const denso_cobotta_gripper::GripperMoveGoalConstPtr& goal);
  void actionCancel(void);
  void gripperCommandActionFeedback(void);
  void gripperMoveActionFeedback(void);

  // Action servers.
  std::shared_ptr<actionlib::SimpleActionServer<denso_cobotta_gripper::GripperMoveAction>> as_gripper_move_;
  std::shared_ptr<actionlib::SimpleActionServer<control_msgs::GripperCommandAction>> as_gripper_cmd_;

  // Publisher
  ros::Publisher pub_joint_state_;
};

}  // namespace denso_cobotta_gripper

#endif  // GRIPPER_PRALLEL_H_
