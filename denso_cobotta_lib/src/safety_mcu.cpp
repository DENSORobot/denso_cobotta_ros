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
#include <cerrno>
#include <sys/ioctl.h>

#include <ros/ros.h>

#include "denso_cobotta_lib/cobotta_common.h"
#include "denso_cobotta_lib/safety_mcu.h"

namespace denso_cobotta_lib
{
namespace cobotta
{
const char* SafetyMcu::TAG = "SafetyMcu";

/**
 * Constructs a SafetyMcu object.
 * @param parent cobotta object
 */
SafetyMcu::SafetyMcu(const std::shared_ptr<Cobotta>& parent)
{
  this->parent_ = parent;
}

/**
 * Constructs a SafetyMcu object.
 * @param parent cobotta object
 * @param state SafetyMCU state
 * @param state_queue number of SafetyMCU queue
 * @param fatal_error fatal error(Lv5) flag
 * @param error error flag
 * @param emergency_button emergency button
 * @param protective_button protective button
 */
SafetyMcu::SafetyMcu(const std::shared_ptr<Cobotta>& parent,
                     const enum SafetyMcuState state, const int state_queue,
                     const bool fatal_error, const bool error,
                     const bool emergency_button, const bool protective_button)
{
  this->parent_ = parent;
  this->state_ = state;
  this->state_queue_ = state_queue;
  this->fatal_error_ = fatal_error;
  this->error_ = error;
  this->emergency_button_ =  emergency_button;
  this->protective_button_ = protective_button;
}

/**
 * Update my state.
 * @param state
 * @param state_queue
 * @param fatal_error
 * @param error
 * @param emergency_button
 * @param protective_button
 * @return true:changed false:no change
 */
bool SafetyMcu::update(const enum SafetyMcuState state, const int state_queue,
                       const bool fatal_error, const bool error,
                       const bool emergency_button, const bool protective_button)
{
  bool isChanged = false;

  if (this->getState() != state)
  {
    this->state_ = state;
    isChanged = true;
  }
  if (this->getStateQueue() != state_queue)
  {
    this->state_queue_ = state_queue;
    isChanged = true;
  }
  if (this->isFatalError() != fatal_error)
  {
    this->fatal_error_ = fatal_error;
    isChanged = true;
  }
  if (this->isError() != error)
  {
    this->error_ = error;
    isChanged = true;
  }
  if (this->isEmergencyButton() != emergency_button)
  {
    this->emergency_button_ = emergency_button;
    isChanged = true;
  }
  if (this->isProtectiveButton() != protective_button)
  {
    this->protective_button_ = protective_button;
    isChanged = true;
  }
  return isChanged;
}

/**
 * Return possibility to move SafetyMCU state.
 * @return true:possible false:impossible
 */
bool denso_cobotta_lib::cobotta::SafetyMcu::canMoveState()
{
  /*
   * On fatal error, DO NOT USE any commands
   * XXX: watch dog timer
   */
  if (this->isFatalError() || this->getParent()->getDriver()->isFatalError())
  {
    /* Fatal Error */
    Message::putRosConsole(TAG, 0x85400001);
    return false;
  }
  /* emergency */
  if (this->isEmergencyButton())
  {
    /* Turn OFF Emergency-stop and execute the command clear_safe_state. */
    Message::putRosConsole(TAG, 0x81400016);
    return false;
  }
  /* protective */
  if (this->isProtectiveButton())
  {
    /* Turn OFF Protective-stop signal to execute the command. */
    Message::putRosConsole(TAG, 0x81400019);
    return false;
  }
  /* @motor_on */
  if (this->getParent()->getMotor()->isRunning())
  {
    /* You cannot execute a command while motor power is ON. */
    Message::putRosConsole(TAG, 0x8140001D);
    return false;

  }
  /* @brake_release */
  if (!this->getParent()->getBrake()->isLocked())
  {
    /* Lock the brake to execute the command. */
    Message::putRosConsole(TAG, 0x8140001B);
    return false;
  }
  return true;
}

/**
 * [sync] Move the state of SafetyMCU to standby
 *   On error(100ms sleep) -> SAFETY_SEND(standby) -> 500ms sleep
 * @exception CobottaException An error defined by COBOTTA
 * @exception RuntimeError An other error
 */
void SafetyMcu::moveToStandby() throw(CobottaException, std::runtime_error)
{
  /* check */
  if (!this->canMoveState())
  {
    // It can not execute in the current state.
    throw CobottaException(0x0F20FFFF);
  }
  if (this->getState() == SafetyMcuState::STANDBY)
    return;
  if (this->isNormal())
  {
    /* Invalid parameter */
    throw CobottaException(0x8340005A);
  }

  ROS_INFO("%s: Being in a standby...", TAG);
  /* waiting for zero of state queue */
  if (this->getStateQueue())
  {
    while (this->getStateQueue())
    {
      ros::Duration(cobotta_common::getPeriod()).sleep();
    }
  }

  /* start */
  if (this->isError() || this->isFatalError())
  {
    /* moving time: 100ms */
    ros::Duration(0.1).sleep();
  }
  this->writeHwState(this->getParent()->getFd(), SafetyMcuCommand::STANDBY);
  /* moving time: 500ms */
  ros::Duration(0.5).sleep();
  /* sync */
  while (this->getState() != SafetyMcuState::STANDBY)
  {
    if (this->isEmergencyButton() || this->isProtectiveButton())
      throw CobottaException(0x83201F83); /* Operation failed */
    if (this->isFatalError())
      throw CobottaException(0x854000F0); /* Fatal error occurred. */

    ros::Duration(cobotta_common::getPeriod()).sleep();
  }
  ROS_INFO("%s: ...It has been in a standby state.", TAG);
}

/**
 * [sync] Move the state of SafetyMCU to normal
 *   500ms sleep -> SAFETY_SEND(normal) -> 24ms sleep
 * @exception CobottaException An error defined by COBOTTA
 * @exception RuntimeError An other error
 */
void SafetyMcu::moveToNormal() throw(CobottaException, std::runtime_error)
{
  /* check */
  if (!this->canMoveState())
  {
    // It can not execute in the current state.
    throw CobottaException(0x0F20FFFF);
  }

  /* @normal */
  if (this->getState() == SafetyMcuState::NORMAL)
    return;

  /* safe state -> standby */
  if (this->getState() == SafetyMcuState::SAFE_STATE)
  {
    this->moveToStandby();
  }
  /* standby -> normal */
  ROS_INFO("%s: Being in a normal...", TAG);
  /* moving time: 500ms */
  ros::Duration(0.5).sleep();
  /* move */
  this->writeHwState(this->getParent()->getFd(), SafetyMcuCommand::NORMAL);
  /* moving time: 24ms */
  ros::Duration(0.024).sleep();
  /* sync */
  while (this->getState() != SafetyMcuState::NORMAL)
  {
    if (this->isEmergencyButton() || this->isProtectiveButton())
      throw CobottaException(0x83201F83); /* Operation failed */
    if (this->isFatalError() || this->isError())
      throw CobottaException(0x83201F83); /* Operation failed */

    ros::Duration(cobotta_common::getPeriod()).sleep();
  }
  ROS_INFO("%s: ...It has been in a normal state.", TAG);
}

/**
 * [sync] FORCE Move the state of SafetyMCU to standby
 *   On error(100ms sleep) -> SAFETY_SEND(standby) -> 500ms sleep
 * @exception CobottaException An error defined by COBOTTA
 * @exception RuntimeError An other error
 */
void SafetyMcu::forceMoveToStandby() throw(CobottaException, std::runtime_error)
{
  ROS_INFO("%s: To clear watchdog-timer error...", TAG);

  /* moving time: 100ms */
  ros::Duration(0.1).sleep();

  this->writeHwState(this->getParent()->getFd(), SafetyMcuCommand::STANDBY);
  /* moving time: 500ms */
  ros::Duration(0.5).sleep();
  /* sync */
  while (this->getState() != SafetyMcuState::STANDBY)
  {
    if (this->isEmergencyButton() || this->isProtectiveButton())
      throw CobottaException(0x83201F83); /* Operation failed */
    if (this->isFatalError())
      throw CobottaException(0x854000F0); /* Fatal error occurred. */

    ros::Duration(cobotta_common::getPeriod()).sleep();
  }
  ROS_INFO("%s: ...Done.", TAG);
}

/**
 * Dequeue a state queue of SafetyMCU.
 * @return On success, main_code & sub_code returned. On no queues, return zero.
 * @exception CobottaException An error defined by COBOTTA
 * @exception RuntimeError An other error
 */
struct StateCode SafetyMcu::dequeue() throw(CobottaException, std::runtime_error)
{
  return this->readHwQueue(this->getParent()->getFd());
}

/**
 * [ASYNC] COBOTTA_IOCTL_SAFETY_SEND
 * @param fd file descriptor
 * @param command 0:SafeState->Standby 2:Standby->Normal
 * @exception CobottaException An error defined by COBOTTA
 * @exception RuntimeError An other error
 */
void SafetyMcu::writeHwState(const int fd, const enum SafetyMcuCommand command) throw(CobottaException, std::runtime_error)
{
  int ret;
  IOCTL_DATA_SAFETY_SEND dat;
  int myerrno;

  memset(&dat, 0, sizeof(dat));
  errno = 0;
  dat.code = (uint32_t)command;
  ret = ioctl(fd, COBOTTA_IOCTL_SAFETY_SEND, &dat);
  myerrno = errno;
  ROS_DEBUG("%s: COBOTTA_IOCTL_SAFETY_SEND ret=%d errno=%d result=%lX",
            TAG, ret, myerrno, dat.result);
  if (ret)
  {
    ROS_ERROR("%s: ioctl(COBOTTA_IOCTL_SAFETY_SEND): %s",
              TAG, std::strerror(myerrno));
    if (myerrno == ECANCELED)
    {
      throw CobottaException(dat.result);
    }
    throw std::runtime_error(std::strerror(myerrno));
  }
}

/**
 * [ASYNC] COBOTTA_IOCTL_SAFETY_CHECKSTATE
 * @param fd file descriptor
 * @return error code
 * @exception CobottaException An error defined by COBOTTA
 * @exception RuntimeError An other error
 */
struct StateCode SafetyMcu::readHwQueue(const int fd) throw(CobottaException, std::runtime_error)
{
  int ret;
  IOCTL_DATA_SAFETY_CHECKSTATE dat;
  int myerrno;

  memset(&dat, 0, sizeof(dat));
  errno = 0;
  ret = ioctl(fd, COBOTTA_IOCTL_SAFETY_CHECKSTATE, &dat);
  myerrno = errno;
  ROS_DEBUG("%s: COBOTTA_IOCTL_SAFETY_CHECKSTATE ret=%d errno=%d result=%lX",
            TAG, ret, myerrno, dat.result);
  if (ret)
  {
    ROS_ERROR("%s: ioctl(COBOTTA_IOCTL_SAFETY_CHECKSTATE): %s",
              TAG, std::strerror(myerrno));
    if (myerrno == ECANCELED)
    {
      throw CobottaException(dat.result);
    }
    throw std::runtime_error(std::strerror(myerrno));
  }
  return StateCode{dat.err.error_code, dat.err.sub_code};
}

bool SafetyMcu::isNormal() const
{
  if (this->getState() == SafetyMcuState::NORMAL)
    return true;
  return false;
}
bool SafetyMcu::isSafeState() const
{
  if (this->getState() == SafetyMcuState::SAFE_STATE)
    return true;
  return false;
}

bool SafetyMcu::isEmergencyButton() const
{
  return emergency_button_;
}

bool SafetyMcu::isError() const
{
  return error_;
}

bool SafetyMcu::isFatalError() const
{
  return fatal_error_;
}

bool SafetyMcu::isProtectiveButton() const
{
  return protective_button_;
}

/**
 * Return SafetyMCU state [safe state/standby/normal]
 * @return enum SafetyMcuState
 */
enum SafetyMcuState SafetyMcu::getState() const
{
  return state_;
}

int SafetyMcu::getStateQueue() const
{
  return state_queue_;
}

std::shared_ptr<Cobotta> SafetyMcu::getParent() const
{
  return parent_;
}

} /* namespace cobotta */
} /* namespace denso_cobotta_lib */

