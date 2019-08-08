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
#include <errno.h>
#include <mutex>
#include "denso_cobotta_gripper/gripper_base.h"

namespace denso_cobotta_gripper
{
using namespace cobotta_common;
using namespace denso_cobotta_lib::cobotta;

GripperBase::GripperBase()
{
  gripper_type_ = GripperType::Undefined;

  current_cmd_position_ = 0.0;
  current_cmd_velocity_ = 0.0;
  current_target_position_ = 0.0;
  current_speed_percentage_ = 0.0;
  current_effort_ = 0.0;
  start_position_ = 0.0;

  current_position_ = 0.0;
  max_soft_limit_ = 0.0;
  min_soft_limit_ = 0.0;
  max_velocity_ = 0.0;
  max_acceleration_ = 0.0;
  max_speed_percentage_ = 0.0;
  min_speed_percentage_ = 0.0;
  max_effort_ = 0.0;
  min_effort_ = 0.0;
  coeff_outpos_to_pulse_ = 0.0;
  coeff_effort_to_torque_ = 0.0;

  fd_ = -1;
  motor_on_ = false;
  moving_ = false;
}

bool GripperBase::initialize(ros::NodeHandle& nh)
{
  bool success;
  // Initialie subscribers.
  success = initSubscribers(nh);
  if (!success)
  {
    return false;
  }

  // Load gripper configuration parameters.
  success = loadConfigParams(nh);
  if (!success)
  {
    return false;
  }

  // Open device file.
  success = openDeviceFile();
  if (!success)
  {
    return false;
  }

  // Verify gripper type.
  std::string gripper_type;
  if (!nh.getParam("gripper_type", gripper_type))
  {
    ROS_ERROR("Failed to get gripper_type.");
    return false;
  }
  if (!this->verifyGripperType(gripper_type))
    return false;

  return true;
}

void GripperBase::checkMotorState(void)
{
  // motor status
  try
  {
    int motor_state = Motor::readHw(fd_);
    ROS_INFO("motor_state is %d", motor_state);
    if (motor_state)
      this->setMotorOn(true);
    else
      this->setMotorOn(false);
  }
  catch (const CobottaException& e)
  {
    Message::putRosConsole(TAG, e);
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
}

bool GripperBase::openDeviceFile()
{
  int myerrno;

  errno = 0;
  fd_ = open(cobotta_common::PATH_DEVFILE, O_RDWR);
  myerrno = errno;
  if (fd_ < 0)
  {
    ROS_ERROR("open(%s): %s", cobotta_common::PATH_DEVFILE, std::strerror(myerrno));
    return false;
  }

  return true;
}

bool GripperBase::verifyGripperType(const std::string& gripper_str)
{
  try
  {
    int gripper_type = static_cast<int>(Gripper::convertGripperType(gripper_str));
    std::array<uint16_t, JOINT_MAX + 1> send_values = {};
    std::array<uint16_t, JOINT_MAX + 1> recv_values;
    Driver::writeHwAcyclicCommAll(fd_, GRIPPER_TYPE_ADDRESS, send_values, recv_values);

    if (recv_values[8] != gripper_type)
    {
      ROS_ERROR("Invalid the gripper type. H/W parameter is %d although the argument is %s(%d).",
                recv_values[8], gripper_str.c_str(), gripper_type);
      return false;
    }
    ROS_INFO("Gripper type is %s.", gripper_str.c_str());
  }
  catch (const CobottaException& e)
  {
    Message::putRosConsole(TAG, e);
    return false;
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    return false;
  }
  return true;
}

bool GripperBase::loadConfigParams(ros::NodeHandle& nh)
{
  if (!nh.getParam("max_soft_limit", max_soft_limit_))
  {
    ROS_ERROR("Failed to load max_soft_limit.");
    return false;
  }

  if (!nh.getParam("min_soft_limit", min_soft_limit_))
  {
    ROS_ERROR("Failed to load min_soft_limit.");
    return false;
  }

  if (!nh.getParam("max_velocity", max_velocity_))
  {
    ROS_ERROR("Failed to load max_velocity.");
    return false;
  }

  if (!nh.getParam("max_acceleration", max_acceleration_))
  {
    ROS_ERROR("Failed to load max_acceleration.");
    return false;
  }

  if (!nh.getParam("max_speed_percentage", max_speed_percentage_))
  {
    ROS_ERROR("Failed to load max_speed_percentage.");
    return false;
  }

  if (!nh.getParam("min_speed_percentage", min_speed_percentage_))
  {
    ROS_ERROR("Failed to load min_speed_percentage.");
    return false;
  }

  if (!nh.getParam("max_effort", max_effort_))
  {
    ROS_ERROR("Failed to load max_effort.");
    return false;
  }

  if (!nh.getParam("min_effort", min_effort_))
  {
    ROS_ERROR("Failed to load min_effort.");
    return false;
  }

  if (!nh.getParam("coeff_outpos_to_pulse", coeff_outpos_to_pulse_))
  {
    ROS_ERROR("Failed to load coeff_outpos_to_pulse.");
    return false;
  }

  if (!nh.getParam("coeff_effort_to_torque", coeff_effort_to_torque_))
  {
    ROS_ERROR("Failed to load coeff_effort_to_torque.");
    return false;
  }

  return true;
}

bool GripperBase::initSubscribers(ros::NodeHandle& nh)
{
  sub_robot_state_ = nh.subscribe("robot_state", 64, &GripperBase::subRobotState, this);
  sub_grasp_state_ = nh.subscribe("gripper_state", 1, &GripperBase::subGraspState, this);
  return true;
}

bool GripperBase::stopMove(void)
{
  std::lock_guard<std::mutex> gripper_lock(move_lock_);
  double slowdown_length = std::pow(current_cmd_velocity_, 2) * 0.5 / max_acceleration_;
  double direction = ((current_target_position_ - start_position_) >= 0.0) ? 1.0 : -1.0;
  if (direction > 0)
  {
    current_target_position_ = current_cmd_position_ + slowdown_length;
  }
  else
  {
    current_target_position_ = current_cmd_position_ - slowdown_length;
  }

  return true;
}

bool GripperBase::calcGripperCommand(void)
{
  if (!move_lock_.try_lock())
  {
    return true;
  }
  const double max_cmd_vel = max_velocity_ * current_speed_percentage_ * 0.01;
  const double direction = ((current_target_position_ - start_position_) >= 0.0) ? 1.0 : -1.0;
  const double vel_delta_increment = max_acceleration_ * getPeriod().toSec();

  // Set target velocity.
  const double position_error = std::abs(current_target_position_ - current_cmd_position_);
  const double slowdown_length = std::pow(current_cmd_velocity_, 2) * 0.5 / max_acceleration_;
  double target_velocity;
  if (position_error > slowdown_length)
  {
    target_velocity = direction * max_cmd_vel;
  }
  else
  {
    target_velocity = direction * vel_delta_increment;
  }

  // Set command velocity.
  if (current_cmd_velocity_ < target_velocity)
  {
    current_cmd_velocity_ += vel_delta_increment;
    if (current_cmd_velocity_ > target_velocity)
    {
      current_cmd_velocity_ = target_velocity;
    }
  }
  else if (current_cmd_velocity_ > target_velocity)
  {
    current_cmd_velocity_ -= vel_delta_increment;
    if (current_cmd_velocity_ < target_velocity)
    {
      current_cmd_velocity_ = target_velocity;
    }
  }

  // Set command position.
  // While accelerating or deaccelerating, adjust to fit trapezoid pattern.
  current_cmd_position_ += (current_cmd_velocity_ * getPeriod().toSec());
  if (current_cmd_velocity_ != target_velocity)
  {
    if (current_cmd_velocity_ <= target_velocity)
    {
      current_cmd_position_ -= vel_delta_increment * getPeriod().toSec() * 0.5;
    }
    else
    {
      current_cmd_position_ += vel_delta_increment * getPeriod().toSec() * 0.5;
    }
  }

  if (((direction >= 0.0) && (current_cmd_position_ >= current_target_position_)) ||
      ((direction < 0.0) && (current_cmd_position_ <= current_target_position_)))
  {
    // Reached the target position.
    // Stop moving.
    current_cmd_velocity_ = 0.0;
    current_cmd_position_ = current_target_position_;
    moving_ = false;
  }
  else
  {
    // Still not reach the target postion.
    // Keep moving.
    moving_ = true;
  }

  move_lock_.unlock();

  return true;
}

bool GripperBase::initCurPos(void)
{
  return this->recvEncoderData();
}

/**
 * [ASYNC] Send to stay here.
 */
void GripperBase::sendStayHere(int fd)
{
  SRV_COMM_SEND send_data{ 0 };
  send_data.arm_no = 1;
  send_data.discontinuous = 0;
  send_data.disable_cur_lim = 0;
  send_data.stay_here = 1;
  try
  {
    Driver::writeHwUpdate(fd, send_data);
  }
  catch (const std::exception& e)
  {
    // ROS_ERROR_STREAM(e.what());
  }
}

bool GripperBase::recvEncoderData(void)
{
  SRV_COMM_RECV recv_data{ 0 };
  try
  {
    recv_data = Driver::readHwEncoder(fd_, 1);
  }
  catch (const CobottaException& e)
  {
    Message::putRosConsole(TAG, e);
    return false;
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    return false;
  }

  current_position_ = recv_data.encoder[0] / coeff_outpos_to_pulse_;

  return true;
}

void GripperBase::subRobotState(const denso_cobotta_driver::RobotState& msg)
{
  switch (msg.state_code)
  {
    case 0x0f200201:  // motor on
      this->setMotorOn(true);
      ROS_DEBUG("motor on");
      break;
    case 0x0f200202:  // motor off
      this->setMotorOn(false);
      ROS_DEBUG("motor off");
      break;
  }
  ROS_DEBUG("msg received. ArmNo=%d code=0x%08X sub=0x%08X", msg.arm_no, msg.state_code, msg.state_subcode);
}

// @return true:on false:off
bool GripperBase::isMotorOn(void) const
{
  return this->motor_on_;
}

void GripperBase::setMotorOn(bool status)
{
  this->motor_on_ = status;
}

void GripperBase::subGraspState(const std_msgs::Bool::ConstPtr& msg)
{
  this->grasp_state_ = msg->data;
}

bool GripperBase::isGraspState() const
{
  return grasp_state_;
}

bool GripperBase::isBusy(void)
{
  return this->moving_;
}

int GripperBase::getFd() const
{
  return this->fd_;
}

} /* namespace denso_cobotta_gripper */

