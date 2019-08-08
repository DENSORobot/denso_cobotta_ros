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
#include <memory>
#include <ros/ros.h>

#include "denso_cobotta_lib/cobotta_common.h"
#include "denso_cobotta_lib/gripper.h"
#include "denso_cobotta_gripper/denso_cobotta_gripper.h"
#include "denso_cobotta_gripper/gripper_base.h"
#include "denso_cobotta_gripper/gripper_parallel.h"
#include "denso_cobotta_gripper/gripper_vacuum.h"

using namespace denso_cobotta_gripper;
using namespace denso_cobotta_lib::cobotta;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "denso_cobotta_gripper");
  ros::NodeHandle nh;

  std::unique_ptr<denso_cobotta_gripper::GripperBase> gripper;
  std::string gripper_type;
  if (!nh.getParam("gripper_type", gripper_type))
  {
    ROS_ERROR("Failed to get gripper_type.");
    return 1;
  }

  switch (Gripper::convertGripperType(gripper_type))
  {
    case GripperType::Parallel:
      gripper = std::make_unique<GripperParallel>();
      break;
    case GripperType::Vacuum:
      gripper = std::make_unique<GripperVacuum>();
      break;
    case GripperType::None:
      ROS_ERROR("denso_cobotta_gripper was launched although gripper_type is none.");
      return 1;
    default:
      ROS_ERROR("denso_cobotta_gripper was launched although gripper_type %s is not specified.",
                gripper_type.c_str());
      return 1;
  }

  bool success = gripper->initialize(nh);
  if (!success)
  {
    ROS_ERROR_STREAM("Failed to initialize denso_cobotta_gripper.");
    return 1;
  }
  ROS_INFO_STREAM("Success to initialize denso_cobotta_gripper.");

  gripper->checkMotorState();

  ros::AsyncSpinner spinner(1);
  ros::Rate rate(1.0 / cobotta_common::COMMAND_CYCLE);
  spinner.start();

  while (ros::ok())
  {
    success = gripper->read();
    if (!success)
    {
      break;
    }

    if (gripper->isMotorOn())
    {
      success = gripper->update();
      if (!success)
      {
        break;
      }

      success = gripper->write();
      if (!success)
      {
        break;
      }
    }
    rate.sleep();
  }
  spinner.stop();

  gripper->sendStayHere(gripper->getFd());
  ROS_INFO("DensoCobotttaGripper has stopped.");
  return 0;
}
