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

#include "denso_cobotta_control/denso_cobotta_hw.h"

namespace denso_cobotta_control
{
using namespace cobotta_common;

const double DensoCobottaHW::GEAR_RATIOS[] = { 207.563, 693.862, 329.232, 295.532, 283.218, 283.218 };
const int DensoCobottaHW::DIRECTION[] = { 1, 1, -1, -1, 1, -1 };

DensoCobottaHW::DensoCobottaHW()
{
  memset(cmd_, 0, sizeof(cmd_));
  memset(pos_, 0, sizeof(pos_));
  memset(vel_, 0, sizeof(vel_));
  memset(eff_, 0, sizeof(eff_));
  memset(coeff_pulse_to_outpos_, 0, sizeof(coeff_pulse_to_outpos_));
  memset(coeff_outpos_to_pulse_, 0, sizeof(coeff_outpos_to_pulse_));
}

DensoCobottaHW::~DensoCobottaHW()
{
}

bool DensoCobottaHW::initialize(ros::NodeHandle& nh)
{
  // Subscribers. (Start until motor_on)
  motor_on_ = false;
  sub_robot_state_ = nh.subscribe("robot_state", 64, &DensoCobottaHW::subRobotStateCB, this);

  // Open cobotta device file
  errno = 0;
  fd_ = open(PATH_DEVFILE.c_str(), O_RDWR);
  if (fd_ < 0)
  {
    ROS_ERROR("open(%s): %s", PATH_DEVFILE.c_str(), std::strerror(errno));
    return false;
  }

  // Calculate the coefficients for converting motor pulse[pls] <=> robot joint position[rad].
  calcPulseConversionCoeff();

  // Get the current encoder value and initialize joint positions.
  getEncoderData();

  // Register hardware interface.
  for (int i = 0; i < CONTROL_JOINT_MAX; i++)
  {
    std::stringstream ss;
    ss << "joint_" << i + 1;

    hardware_interface::JointStateHandle state_handle(ss.str(), &pos_[i], &vel_[i], &eff_[i]);
    jntStInterface_.registerHandle(state_handle);
    hardware_interface::JointHandle pos_handle(jntStInterface_.getHandle(ss.str()), &cmd_[i]);
    posJntInterface_.registerHandle(pos_handle);

    cmd_[i] = pos_[i];
  }
  registerInterface(&posJntInterface_);
  registerInterface(&jntStInterface_);

  return true;
}

bool DensoCobottaHW::Read(ros::Time time, ros::Duration period)
{
  bool success = getEncoderData();
  if (!success)
  {
    return false;
  }
  return true;
}

bool DensoCobottaHW::Write(ros::Time time, ros::Duration period)
{
  bool success = setServoUpdateData();
  if (!success)
  {
    return false;
  }
  return true;
}

bool DensoCobottaHW::setServoUpdateData()
{
  upd_.Send.ArmNo = 0;
  upd_.Send.Discontinuous = 0;
  upd_.Send.DisableCurLim = 0;
  upd_.Send.StayHere = 0;

  for (int i = 0; i < CONTROL_JOINT_MAX; i++)
  {
    upd_.Send.Position[i] = coeff_outpos_to_pulse_[i] * cmd_[i];
    upd_.Send.CurrentLimit[i] = 0x0FFF;
    upd_.Send.CurrentOffset[i] = 0;
  }

  errno = 0;
  int ret = ioctl(fd_, COBOTTA_IOCTL_SRV_UPDATE, &upd_);
  if (ret != 0)
  {
    ROS_ERROR("ioctl(SRV_UPDATE IOCTL):%s (ret=%d errno=%d)", std::strerror(errno), ret, errno);
    return false;
  }

  if (upd_.Recv.Result == 0)
  {
    // Servo updating without delay.
  }
  else if (upd_.Recv.Result == 0x0F408101)
  {
    // The current number of commands in buffer is 11.
    // To avoid buffer overflow, sleep 8 msec.
    ros::Duration(0.008).sleep();
  }
  else
  {
    ROS_ERROR_THROTTLE(1,
                       "DensoCobottaHW: Command buffer overflow or other errors."
                       " (res=0x%08x state=0x%04x num=%d)",
                       upd_.Recv.Result, (upd_.Recv.BuffState & 0xffff0000) >> 16, (upd_.Recv.BuffState & 0x0ffff));
    return false;
  }

  return true;
}

bool DensoCobottaHW::getEncoderData()
{
  static IOCTL_DATA_GETENC encData_;
  encData_.Arm = 0;
  errno = 0;
  int ret = ioctl(fd_, COBOTTA_IOCTL_SRV_GETENC, &encData_);
  if (ret != 0)
  {
    ROS_ERROR("ioctl(SRV_GETENC IOCTL): %s (ret=%d errno=%d)", std::strerror(errno), ret, errno);
    return false;
  }

  // Update current position.
  for (int i = 0; i < CONTROL_JOINT_MAX; i++)
  {
    pos_[i] = coeff_pulse_to_outpos_[i] * encData_.Recv.Encoder[i];
  }
  return true;
}

void DensoCobottaHW::calcPulseConversionCoeff()
{
  for (int i = 0; i < CONTROL_JOINT_MAX; i++)
  {
    // Motor pulse => Robot joint position.
    coeff_pulse_to_outpos_[i] = DIRECTION[i] * (1.0 / 1024.0) * 2.0 * M_PI / GEAR_RATIOS[i];
    // Robot joint Position => motor pulse.
    coeff_outpos_to_pulse_[i] = DIRECTION[i] * GEAR_RATIOS[i] * (1.0 / (2.0 * M_PI)) * 1024.0;
  }
}

void DensoCobottaHW::subRobotStateCB(const denso_cobotta_driver::RobotState& msg)
{
  switch (msg.state_code)
  {
    case 0x0f200201: // motor on
      motor_on_ = true;
      ROS_DEBUG("DensoCobottaHW: motor on");
      break;
    case 0x0f200202: // motor off
      motor_on_ = false;
      ROS_DEBUG("DensoCobottaHW: motor off");
      break;
  }
  ROS_DEBUG("DensoCobottaHW: msg received. ArmNo=%d code=0x%08X sub=0x%08X",
            msg.arm_no, msg.state_code, msg.state_subcode);
}

// @return ture:on false:off
bool DensoCobottaHW::isMotorOn(void)
{
  return motor_on_;
}

}  // namespace denso_cobotta_control
