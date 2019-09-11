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
#include "denso_cobotta_gripper/gripper_vacuum.h"
#include "denso_cobotta_lib/driver.h"
#include "denso_cobotta_gripper/VacuumMoveAction.h"

namespace denso_cobotta_gripper
{
using namespace denso_cobotta_lib::cobotta;

GripperVacuum::GripperVacuum()
{
  ROS_INFO("GripperVacuum loading...");
  gripper_type_ = GripperType::Vacuum;
}

bool GripperVacuum::initialize(ros::NodeHandle& nh)
{
  bool success = GripperBase::initialize(nh);
  if (!success)
  {
    return false;
  }

  success = initActionServers(nh);
  if (!success)
  {
    return false;
  }

  success = GripperBase::initCurPos();
  if (!success)
  {
    return false;
  }

  start_position_ = current_position_;
  current_target_position_ = current_position_;
  current_cmd_position_ = current_position_;
  current_cmd_velocity_ = 0.0;
  current_speed_percentage_ = 0.0;
  moving_ = false;

  return true;
}

bool GripperVacuum::read()
{
  if (!GripperBase::recvEncoderData())
  {
    return false;
  }

  return true;
}

bool GripperVacuum::write()
{
  SRV_COMM_SEND send_data{ 0 };
  send_data.arm_no = 1;
  send_data.discontinuous = 0;
  send_data.disable_cur_lim = 0;
  send_data.stay_here = 0;

  send_data.position[0] = current_cmd_position_ * coeff_outpos_to_pulse_;
  // For vacuum gripper, we fix the maximum current limit.
  send_data.current_limit[0] = 0x0FFF;
  send_data.current_offset[0] = 0;

  try
  {
    struct DriverCommandInfo info = Driver::writeHwUpdate(fd_, send_data);
    if (info.result == 0x0F408101)
    {
      // The current number of commands in buffer is 11.
      // To avoid buffer overflow, sleep 8 msec
      ros::Duration(cobotta_common::COMMAND_SHORT_BREAK).sleep();
    }
    else if (info.result == 0x84400502)
    {
      // buffer full
      ROS_WARN("Command buffer overflow...");
      ros::Duration(cobotta_common::COMMAND_LONG_BREAK).sleep();
    }
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    return false;
  }

  return true;
}

bool GripperVacuum::update()
{
  GripperBase::calcGripperCommand();

  return true;
}

bool GripperVacuum::initActionServers(ros::NodeHandle& nh)
{
  as_vacuum_move_ = std::make_shared<actionlib::SimpleActionServer<VacuumMoveAction> >(
      nh, "vacuum_move",
      std::bind(&GripperVacuum::vacuumMoveActionGoal, this, std::placeholders::_1), false);
  as_vacuum_move_->registerPreemptCallback(std::bind(&GripperVacuum::actionCancel, this));
  as_vacuum_move_->start();

  return true;
}

/**
 * @param target_position blow:minimum value suction:maximum value
 * @param speed_percentage speed_percentage is the same as power_percentage
 */
bool GripperVacuum::initGripperMove(const double& target_position, const double& speed_percentage)
{
  // For vacuum gripper, we check only speed percentage.
  if ((speed_percentage < min_speed_percentage_) || (speed_percentage > max_speed_percentage_))
  {
    ROS_ERROR("DensoCobottaGripper: speed is out of range(%lf). Min is %lf. Max is %lf.", speed_percentage,
              min_speed_percentage_, max_speed_percentage_);
    return false;
  }

  if (!updateVacuumDetectParameter(target_position > 0 ? speed_percentage : 100))
  {
    return false;
  }
  // Set command parameters.
  {
    std::lock_guard<std::mutex> gripper_lock(move_lock_);
    start_position_ = current_cmd_position_;
    current_speed_percentage_ = speed_percentage;
    current_effort_ = 0.0;
    current_target_position_ = target_position;
    moving_ = true;
  }

  return true;
}

bool GripperVacuum::updateVacuumDetectParameter(double speed_percentage)
{
  try
  {
    const double DETECT_SLOPE = -0.1;
    const double DETECT_INTERCEPT = 1.8;
    const double DETECT_MAGNIFICATION = 10;
    std::array<uint16_t, JOINT_MAX + 1> send_values = { 0 };
    std::array<uint16_t, JOINT_MAX + 1> recv_values = { 0 };
    Driver::writeHwAcyclicCommAll(fd_, cobotta_common::VACUUM_DETECT_ADDRESS, send_values, recv_values);
    send_values = recv_values;
    send_values[send_values.size() - 1] = (uint16_t)(DETECT_MAGNIFICATION * (DETECT_SLOPE * speed_percentage + DETECT_INTERCEPT));
    Driver::writeHwAcyclicCommAll(fd_, cobotta_common::VACUUM_DETECT_ADDRESS | cobotta_common::ACYCLIC_WRITE_MASK,
                                  send_values, recv_values);
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

bool GripperVacuum::vacuumMoveActionGoal(const denso_cobotta_gripper::VacuumMoveGoalConstPtr& goal)
{
  ROS_DEBUG("vacuumMoveActionGoal: direction=%d power_percentage=%lf",
            goal->direction, goal->power_percentage);

  if (!GripperBase::isMotorState())
  {
    Message::putRosConsole(TAG, 0x81400014);
    as_vacuum_move_->setAborted();
    return false;
  }

  VacuumMoveResult result;
  VacuumMoveDirection direction = static_cast<VacuumMoveDirection>(goal->direction);

  // When the power percentage is 0, request stop.
  if (std::abs(goal->power_percentage) <= std::numeric_limits<double>::epsilon())
  {
    direction = VacuumMoveDirection::Stop;
  }

  double target_position;
  if (direction == VacuumMoveDirection::Blow)
  {
    target_position = std::numeric_limits<double>::lowest();
  }
  else if (direction == VacuumMoveDirection::Suction)
  {
    target_position = std::numeric_limits<double>::max();
  }
  else if (direction == VacuumMoveDirection::Stop)
  {
    GripperBase::stopMove();
    result.success = true;
    as_vacuum_move_->setSucceeded(result);
    return true;
  }
  else
  {
    ROS_ERROR("Vacuum goal direction is invalid: %d", goal->direction);
    as_vacuum_move_->setAborted();
    return false;
  }

  bool success = initGripperMove(target_position, goal->power_percentage);
  if (!success)
  {
    as_vacuum_move_->setAborted();
    return false;
  }

  ros::Rate rate(1.0 / cobotta_common::SERVO_PERIOD);
  while (moving_)
  {
    // Wait until move is complete.
    this->actionFeedback();
    rate.sleep();
  }
  this->actionFeedback();

  if (as_vacuum_move_->isPreemptRequested())
  {
    result.success = false;
    as_vacuum_move_->setPreempted(result);
    return false;
  }
  else if (as_vacuum_move_->isActive())
  {
    result.success = true;
    as_vacuum_move_->setSucceeded(result);
  }
  else
  {
    as_vacuum_move_->setAborted();
    return false;
  }
  return true;
}

void GripperVacuum::actionCancel()
{
  GripperBase::stopMove();
  return;
}

/*
 * The pressure of feedback is not implemented.
 * It needs Linux driver fix.
 */
void GripperVacuum::actionFeedback()
{
  VacuumMoveFeedback feedback;
  feedback.pressure = 0; // Not implemented.
  feedback.stalled = this->isGraspState();
  as_vacuum_move_->publishFeedback(feedback);

  return;
}

bool GripperVacuum::publish()
{
  return true;
}

bool GripperVacuum::subscribe()
{
  return true;
}

} /* namespace denso_cobotta_gripper */

