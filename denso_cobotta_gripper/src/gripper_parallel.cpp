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
#include "denso_cobotta_gripper/gripper_parallel.h"
#include "denso_cobotta_gripper/GripperMoveAction.h"

namespace denso_cobotta_gripper
{
using namespace denso_cobotta_lib::cobotta;

GripperParallel::GripperParallel()
{
  ROS_INFO("GripperParallel loading...");
  gripper_type_ = GripperType::Parallel;
}

bool GripperParallel::initialize(ros::NodeHandle& nh)
{
  bool success = GripperBase::initialize(nh);
  if (!success)
  {
    return false;
  }
  // Publisher
  pub_joint_state_ = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

  // Action server
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

bool GripperParallel::read()
{
  if (!GripperBase::recvEncoderData())
  {
    return false;
  }
  this->publish();

  return true;
}

bool GripperParallel::write()
{
  return GripperParallel::sendServoUpdateData();
}

bool GripperParallel::update()
{
  return GripperBase::calcGripperCommand();
}

bool GripperParallel::publish()
{
  // Publish joint data to joint state publisher.
  sensor_msgs::JointState gripper;
  gripper.header.stamp = ros::Time::now();
  gripper.name.resize(1);
  gripper.name[0] = "joint_gripper";
  gripper.position.resize(1);
  // As we use mimic joint, mutiply by 0.5 to get the intended width.
  gripper.position[0] = current_position_ * 0.5;
  pub_joint_state_.publish(gripper);
}

bool GripperParallel::initActionServers(ros::NodeHandle& nh)
{
  as_gripper_move_ = std::make_shared<actionlib::SimpleActionServer<GripperMoveAction> >(
      nh, "gripper_move", std::bind(&GripperParallel::gripperMoveActionGoal, this, std::placeholders::_1), false);
  as_gripper_move_->registerPreemptCallback(std::bind(&GripperParallel::actionCancel, this));
  as_gripper_move_->start();

  as_gripper_cmd_ = std::make_shared<actionlib::SimpleActionServer<control_msgs::GripperCommandAction> >(
      nh, "gripper_action", std::bind(&GripperParallel::gripperCommandActionGoal, this, std::placeholders::_1),
      false);
  as_gripper_cmd_->registerPreemptCallback(std::bind(&GripperParallel::actionCancel, this));
  as_gripper_cmd_->start();

  return true;
}

bool GripperParallel::initGripperMove(const double& target_position, const double& speed_percentage,
                                      const double& effort)
{
  if ((target_position < min_soft_limit_) || (target_position > max_soft_limit_))
  {
    ROS_ERROR("DensoCobottaGripper: target_position is out of range(%lf). Min is %lf. Max is %lf.", target_position,
              min_soft_limit_, max_soft_limit_);
    return false;
  }

  if ((speed_percentage < min_speed_percentage_) || (speed_percentage > max_speed_percentage_))
  {
    ROS_ERROR("DensoCobottaGripper: speed is out of range(%lf). Min is %lf. Max is %lf.", speed_percentage,
              min_speed_percentage_, max_speed_percentage_);
    return false;
  }

  if ((effort < min_effort_) || (effort > max_effort_))
  {
    ROS_ERROR("DensoCobottaGripper: effort is out of range(%lf). Min is %lf. Max is %lf.", effort, min_effort_,
              max_effort_);
    return false;
  }

  // Set command parameters.
  {
    std::lock_guard<std::mutex> gripper_lock(move_lock_);
    start_position_ = current_position_;
    current_cmd_position_ = start_position_;
    current_speed_percentage_ = speed_percentage;
    current_effort_ = effort;
    /*
     * XXX: As we use mimic joint, mutiply by 2 to get the intended width.
     */
    current_target_position_ = target_position * 2;
    moving_ = true;
  }

  return true;
}

bool GripperParallel::gripperMoveActionGoal(const GripperMoveGoalConstPtr& goal)
{
  ROS_DEBUG("gripperMoveActionGoal: target_position=%lf effort=%f speed=%f",
            goal->target_position, goal->effort, goal->speed);

  if (!GripperBase::isMotorOn())
  {
    /* Turn motor power ON to execute the command. */
    Message::putRosConsole(TAG, 0x81400014);
    as_gripper_move_->setAborted();
    return false;
  }

  double target_position = goal->target_position;
  double effort = goal->effort;
  double speed = goal->speed;

  GripperMoveResult result;
  bool success = initGripperMove(target_position, speed, effort);
  if (!success)
  {
    as_gripper_move_->setAborted();
    return false;
  }

  ros::Rate rate(1.0 / cobotta_common::SERVO_PERIOD);
  while (moving_)
  {
    // Wait until move is complete.
    this->gripperMoveActionFeedback();
    rate.sleep();
  }
  this->gripperMoveActionFeedback();

  // set the action state
  if (as_gripper_move_->isPreemptRequested())
  {
    result.success = false;
    as_gripper_move_->setPreempted(result);
    return false;
  }
  else if (as_gripper_move_->isActive())
  {
    result.success = true;
    as_gripper_move_->setSucceeded(result);
  }
  else
  {
    as_gripper_move_->setAborted();
    return false;
  }

  return true;
}

void GripperParallel::actionCancel()
{
  GripperBase::stopMove();
  return;
}

void GripperParallel::gripperMoveActionFeedback()
{
  /* gripper_move */
  GripperMoveFeedback feedback;
  // As we use mimic joint, mutiply by 0.5 to get the intended width.
  feedback.position = current_position_ * 0.5;
  feedback.stalled = this->isGraspState();
  feedback.reached_goal = !moving_;
  as_gripper_move_->publishFeedback(feedback);

  return;
}

void GripperParallel::gripperCommandActionFeedback()
{
  /* gripper command */
  control_msgs::GripperCommandActionFeedback feedback;
  feedback.feedback.effort = 0;
  // As we use mimic joint, mutiply by 0.5 to get the intended width.
  feedback.feedback.position = current_position_ * 0.5;
  feedback.feedback.stalled = this->isGraspState();
  feedback.feedback.reached_goal = !moving_;
  as_gripper_cmd_->publishFeedback(feedback.feedback);

  return;
}

/**
 * [ASYNC] Send to stay here.
 */
void GripperParallel::sendStayHere(int fd)
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

bool GripperParallel::sendServoUpdateData()
{
  // ArmNo: 0 => J1 to J6.
  // ArmNo: 1 => Gripper
  SRV_COMM_SEND send_data{ 0 };
  send_data.arm_no = 1;
  send_data.discontinuous = 0;
  send_data.disable_cur_lim = 0;
  send_data.stay_here = 0;

  send_data.position[0] = coeff_outpos_to_pulse_ * current_cmd_position_;
  send_data.current_limit[0] = coeff_effort_to_torque_ * current_effort_ * 1000;
  send_data.current_offset[0] = 0;

  try
  {
    struct DriverCommandInfo info = Driver::writeHwUpdate(fd_, send_data);
    // ROS_DEBUG("result:%08X queue:%d stay_here:%d", info.result, info.queue_num, info.stay_here);
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

bool GripperParallel::gripperCommandActionGoal(const control_msgs::GripperCommandGoalConstPtr& goal)
{
  ROS_DEBUG("gripperCommandActionGoal: target_position=%lf max_effort=%lf",
            goal->command.position, goal->command.max_effort);

  if (!GripperBase::isMotorOn())
  {
    /* Turn motor power ON to execute the command. */
    Message::putRosConsole(TAG, 0x81400014);
    as_gripper_cmd_->setAborted();
    return false;
  }

  double target_position = goal->command.position;
  double max_effort = goal->command.max_effort;
  double speed_rate = 20.0;

  /*
   * XXX: MoveIt! send us effort is zero.
   * Round of minimum effort anyway.
   */
  if (max_effort < this->min_effort_) {
    max_effort = this->min_effort_;
    ROS_WARN("gripperCommandActionGoal: Round of effort %f because input value is smaller than minimum.",
             max_effort);
  }

  bool success = initGripperMove(target_position, speed_rate, max_effort);
  // set the action state to succeeded
  if (!success)
  {
    as_gripper_cmd_->setAborted();
    return false;
  }

  ros::Rate rate(1.0 / cobotta_common::SERVO_PERIOD);
  while (moving_)
  {
    // Wait until move is complete.
    this->gripperCommandActionFeedback();
    rate.sleep();
  }
  this->gripperCommandActionFeedback();

  control_msgs::GripperCommandResult result;
  result.effort = 0;
  result.position = this->current_position_;
  result.stalled = this->grasp_state_;
  // set the action state
  if (as_gripper_cmd_->isPreemptRequested())
  {
    result.reached_goal = false;
    as_gripper_cmd_->setPreempted(result);
    return false;
  }
  else if (as_gripper_cmd_->isActive())
  {
    result.reached_goal = true;
    as_gripper_cmd_->setSucceeded(result);
  }
  else
  {
    as_gripper_cmd_->setAborted();
    return false;
  }
  return true;
}

bool GripperParallel::subscribe()
{
  return true;
}

} /* namespace denso_cobotta_gripper */
