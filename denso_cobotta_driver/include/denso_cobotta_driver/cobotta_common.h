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

#ifndef _COBOTTA_COMMON_H_
#define _COBOTTA_COMMON_H_

#include <string>
#include <ros/ros.h>

namespace cobotta_common
{
const static std::string PATH_DEVFILE = "/dev/denso_cobotta";
const static uint32_t CONTROL_JOINT_MAX = 6;
const static uint32_t SRVSTATE_CB_LED = 0x00010101;

static ros::Duration getPeriod()
{
  return ros::Duration(0.008);
}

}  // namespace cobotta_common

#endif  // _COBOTTA_COMMON_H_
