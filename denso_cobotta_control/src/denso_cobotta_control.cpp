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

#include <controller_manager/controller_manager.h>
#include "denso_cobotta_control/denso_cobotta_hw.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "denso_cobotta_control");
  ros::NodeHandle nh;

  denso_cobotta_control::DensoCobottaHW cobotta;

  bool success = cobotta.initialize(nh);
  if (!success)
  {
    ROS_ERROR_STREAM("Failed to initialize denso_cobotta_control.");
    return 1;
  }

  controller_manager::ControllerManager cm(&cobotta, nh);

  ros::Rate rate(1.0 / cobotta_common::getPeriod().toSec());
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Time start = ros::Time::now();
  ros::Duration dt = cobotta_common::getPeriod();

  while (ros::ok())
  {
    ros::Time now = ros::Time::now();

    success = cobotta.Read(now, dt);
    if (!success)
    {
      return 1;
    }
    cm.update(now, dt);

    if (!cobotta.isMotorOn())
    {
      rate.sleep();
      continue;
    }

    success = cobotta.Write(now, dt);
    if (!success)
    {
      return 1;
    }
    ros::Duration(0.001).sleep();
  }
  spinner.stop();

  return 0;
}
