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
#include <signal.h>

#include "denso_cobotta_control/denso_cobotta_hw.h"
#include "denso_cobotta_lib/driver.h"

namespace denso_cobotta_control
{
using namespace cobotta_common;
using namespace denso_cobotta_lib::cobotta;

DensoCobottaHW::DensoCobottaHW()
{
  memset(cmd_, 0, sizeof(cmd_));
  memset(pos_, 0, sizeof(pos_));
  memset(vel_, 0, sizeof(vel_));
  memset(eff_, 0, sizeof(eff_));
}

DensoCobottaHW::~DensoCobottaHW()
{
}

bool DensoCobottaHW::initialize(ros::NodeHandle& nh)
{
  // Subscribers. (Start until motor_on)
  motor_on_ = false;
  reset_ = false;
  sub_robot_state_ = nh.subscribe("robot_state", 64, &DensoCobottaHW::subRobotStateCB, this);

  // Open cobotta device file
  errno = 0;
  fd_ = open(PATH_DEVFILE.c_str(), O_RDWR);
  if (fd_ < 0)
  {
    ROS_ERROR("open(%s): %s", PATH_DEVFILE.c_str(), std::strerror(errno));
    return false;
  }

  // load calset....
  std::ifstream ifs(CALSET_PATH);
  if (ifs.is_open())
  {
    std::string command = "rosparam load " + CALSET_PATH + " /cobotta/pulse_offset";
    std::system(command.c_str());

    if (!nh.getParam("/cobotta/pulse_offset", pulse_offset_) || pulse_offset_.size() != CONTROL_JOINT_MAX)
    {
      ROS_INFO("Cannot purse pulse offest.");
      pulse_offset_.resize(CONTROL_JOINT_MAX);
      for (auto& i : pulse_offset_)
      {
        i = 0;
      }
    }
    else
    {
      ROS_INFO("Success to set pulse offest.");
    }
  }
  else
  {
    ROS_INFO("Cannot read calset file.");
    pulse_offset_.resize(CONTROL_JOINT_MAX);
    for (auto& i : pulse_offset_)
    {
      i = 0;
    }
  }

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

bool DensoCobottaHW::read(ros::Time time, ros::Duration period)
{
  bool success = getEncoderData();
  if (!success)
  {
    return false;
  }
  return true;
}

bool DensoCobottaHW::write(ros::Time time, ros::Duration period)
{
  bool success = setServoUpdateData();
  if (!success)
  {
    return false;
  }
  return true;
}

/**
 * [ASYNC] Send to stay here
 */
void DensoCobottaHW::sendStayHere(int fd)
{
  SRV_COMM_SEND send_data = {0};
  send_data.arm_no = 0;
  send_data.discontinuous = 0;
  send_data.disable_cur_lim = 0;
  send_data.stay_here = 1;

  try
  {
    Driver::writeHwUpdate(fd, send_data);
  }
  catch (const std::exception& e)
  {
    //ROS_ERROR_STREAM(e.what());
  }

}
bool DensoCobottaHW::setServoUpdateData()
{
  SRV_COMM_SEND send_data;
  send_data.arm_no = 0;
  send_data.discontinuous = 0;
  send_data.disable_cur_lim = 0;
  send_data.stay_here = 0;

  for (int i = 0; i < CONTROL_JOINT_MAX; i++)
  {
    send_data.position[i] = ARM_COEFF_OUTPOS_TO_PULSE[i] * cmd_[i] - pulse_offset_[i];
    send_data.current_limit[i] = 0x0FFF;
    send_data.current_offset[i] = 0;
  }

  try
  {
    struct DriverCommandInfo info = Driver::writeHwUpdate(fd_, send_data);
    //ROS_DEBUG("result:%08X queue:%d stay_here:%d", info.result, info.queue_num, info.stay_here);
    if (info.result == 0x0F408101)
    {
      // The current number of commands in buffer is 11.
      // To avoid buffer overflow, sleep 8 msec
      ros::Duration(cobotta_common::getPeriod()).sleep();
    }
    else if (info.result == 0x84400502)
    {
      // buffer full
      ROS_WARN("Command buffer overflow...");
      ros::Duration(cobotta_common::getPeriod() * 2).sleep();
    }
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    return false;
  }
  return true;
}

bool DensoCobottaHW::getEncoderData()
{
  SRV_COMM_RECV recv_data{ 0 };

  try
  {
    recv_data = Driver::readHwEncoder(fd_, 0);
  }
  catch (const CobottaException& e)
  {
    ROS_ERROR_STREAM(e.what());
    return false;
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    return false;
  }

  // Update current position.
  for (int i = 0; i < CONTROL_JOINT_MAX; i++)
  {
    pos_[i] = (1 / ARM_COEFF_OUTPOS_TO_PULSE[i]) * (recv_data.encoder[i] + pulse_offset_[i]);
  }
  return true;
}

void DensoCobottaHW::subRobotStateCB(const denso_cobotta_driver::RobotState& msg)
{
  switch (msg.state_code)
  {
    case 0x0f200201:  // motor on
      motor_on_ = true;
      reset_ = true;
      ROS_DEBUG("motor on");
      break;
    case 0x0f200202:  // motor off
      motor_on_ = false;
      ROS_DEBUG("motor off");
      break;
  }
  ROS_DEBUG("msg received. ArmNo=%d code=0x%08X sub=0x%08X", msg.arm_no, msg.state_code, msg.state_subcode);
}

// @return ture:on false:off
bool DensoCobottaHW::isMotorOn(void)
{
  return motor_on_;
}

bool DensoCobottaHW::shouldReset(void)
{
  if (this->reset_)
  {
    this->reset_ = false;
    return true;
  }
  return false;
}

int DensoCobottaHW::getFd() const
{
  return fd_;
}
} // namespace denso_cobotta_control

