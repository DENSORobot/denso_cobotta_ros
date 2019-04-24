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
#include <array>
#include <ros/ros.h>

namespace cobotta_common
{
const static int DRIVER_VERSION_MAJOR = 1;
const static int DRIVER_VERSION_MINOR = 2;


const static std::string PATH_DEVFILE = "/dev/denso_cobotta";
const static std::string CALSET_PATH = "/tmp/cobotta_offset.yaml";
const static uint32_t CONTROL_JOINT_MAX = 6;
const static double SERVO_PERIOD = 0.008;  //[s]
const static uint32_t SRVSTATE_CB_LED = 0x00010101;
const static std::array<double, CONTROL_JOINT_MAX> ARM_COEFF_OUTPOS_TO_PULSE = { 33827.5138204596,  113081.9899244610,
                                                                                 -53656.4738294077, -48164.2778752878,
                                                                                 46157.4329638175,  -46157.4329638175 };
const static std::array<double, CONTROL_JOINT_MAX> CALSET_POSE = { 150, -60, 140, -170, 135, 170 };  // [deg]
const static std::array<double, CONTROL_JOINT_MAX> ARM_MAX_ACCELERATION = {
  1.5, 1.5, 2.5, 2.5, 2.5, 3.5
};                                                                                                       // [rad/s^2]
const static std::array<double, CONTROL_JOINT_MAX> ARM_MAX_VELOCITY = { 0.3, 0.3, 0.5, 0.5, 0.5, 0.7 };  // [rad/s]
const static std::array<double, CONTROL_JOINT_MAX> OFFSET_LIMIT = { 0.3, 0.3, 0.5, 0.5, 0.5, 0.7 };      // [rad]

static ros::Duration getPeriod()
{
  return ros::Duration(SERVO_PERIOD);
}

}  // namespace cobotta_common

#endif  // _COBOTTA_COMMON_H_
