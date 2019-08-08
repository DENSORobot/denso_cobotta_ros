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
#include "denso_cobotta_lib/cobotta_ioctl.h"

namespace cobotta_common
{
constexpr static int DRIVER_VERSION_MAJOR = 1;
constexpr static int DRIVER_VERSION_MINOR = 2;

constexpr const static char* PATH_DEVFILE = "/dev/denso_cobotta";
constexpr const static char* TEMP_PARAMS_PATH = "/tmp/cobotta_parameters.yaml";
constexpr static uint32_t CONTROL_JOINT_MAX = 6;
constexpr static double SERVO_PERIOD = 0.008;         //[s]
constexpr static double DRIVER_UPDATE_PERIOD = 0.01;  //[s]
constexpr static uint32_t SRVSTATE_CB_LED = 0x00010101;
constexpr static std::array<double, CONTROL_JOINT_MAX> ARM_COEFF_OUTPOS_TO_PULSE = { 33827.5138204596,  113081.9899244610,
                                                                                 -53656.4738294077, -48164.2778752878,
                                                                                 46157.4329638175,  -46157.4329638175 };
constexpr static std::array<double, CONTROL_JOINT_MAX> CALSET_POSE = { 150, -60, 140, -170, 135, 170 };      // [deg]
constexpr static std::array<double, CONTROL_JOINT_MAX> HOME_POSE = { 0, 30, 100, 0, 50, 0 };                 // [deg]
constexpr static std::array<double, CONTROL_JOINT_MAX> ARM_MAX_ACCELERATION = { 1., 1., 1., 1., 1., 1. };    // [rad/s^2]
constexpr static std::array<double, CONTROL_JOINT_MAX> ARM_MAX_VELOCITY = { 0.2, 0.2, 0.2, 0.2, 0.2, 0.2 };  // [rad/s]
constexpr static std::array<double, CONTROL_JOINT_MAX> OFFSET_LIMIT = { 0.2, 0.2, 0.2, 0.2, 0.2, 0.2 };      // [rad]
constexpr static uint16_t ACYCLIC_WRITE_MASK = 0x8000;
constexpr static uint16_t POSITION_DEVIATION_ADDRESS = 0x0203;;
constexpr static uint16_t GRIPPER_TYPE_ADDRESS = 0x0800;
constexpr static uint16_t VACUUM_DETECT_ADDRESS = 0x0827;
constexpr static std::array<uint16_t, JOINT_MAX + 1> POSITION_DEVIATION_PARAMS = { 16, 53, 47, 43, 40, 61, 37, 37, 10240 };

static ros::Duration getPeriod()
{
  return ros::Duration(SERVO_PERIOD);
}

constexpr static double COMMAND_CYCLE = 0.004;                  /** Update command rate: 4ms (less than 8ms) */
constexpr static double COMMAND_SHORT_BREAK = SERVO_PERIOD * 2; /** Sleep short time to avoid command overflow */
constexpr static double COMMAND_LONG_BREAK = SERVO_PERIOD * 8;  /** Sleep long time to avoid command overflow */

}  // namespace cobotta_common

#endif  // _COBOTTA_COMMON_H_
