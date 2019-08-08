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

#include <string>
#include <cerrno>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <memory>

#include <ros/ros.h>

#include "denso_cobotta_lib/cobotta_common.h"
#include "denso_cobotta_lib/motor.h"
#include "denso_cobotta_lib/cobotta_ioctl.h"

namespace denso_cobotta_lib
{
namespace cobotta
{
/**
 * Constructs a Motor object.
 * @param parent cobotta object
 */
Motor::Motor(std::shared_ptr<Cobotta> parent)
{
  this->parent_ = parent;
}

/**
 * Constructs a Motor object.
 * @param parent cobotta object
 * @param state motor state
 */
Motor::Motor(std::shared_ptr<Cobotta> parent, const enum MotorState state)
{
  this->parent_ = parent;
  this->state_ = state;
}

/**
 * Update my state.
 * @param state motor state
 * @return true:changed false:no change
 */
bool Motor::update(const enum MotorState state)
{
  if (this->getState() == state)
    return false;

  this->state_ = state;
}

/**
 * @return true: Motor should STOP at STO or errors
 */
bool Motor::shouldStop()
{
  if ((this->getState() == MotorState::SlowDownStop)
      || (this->getState() == MotorState::Off))
    return false;

  /* SafetyMCU state */
  if (this->getParent()->getSafetyMcu()->isSafeState())
    return true;
  /* fatal error */
  if (this->getParent()->getDriver()->isFatalError())
    return true;
  /* XXX: Lv4 error */

  return false;
}

/**
 * @return true:On false:Off/OnProc/SlowDownStop
 */
bool Motor::isRunning(void) const
{
  if (this->getState() == MotorState::On)
    return true;

  return false;
}

/**
 * @return true:yes false:no
 */
bool Motor::canStart(void) const
{
  if (this->getParent()->getSafetyMcu()->isFatalError()
      || this->getParent()->getDriver()->isFatalError())
  {
    // Fatal Error
    Message::putRosConsole(TAG, 0x85400001);
    return false;
  }
  /* emergency button */
  if (this->getParent()->getSafetyMcu()->isEmergencyButton())
  {
    // Turn OFF Emergency-stop and execute the command clear_safe_state.
    Message::putRosConsole(TAG, 0x81400016);
    return false;
  }
  /* protective button */
  if (this->getParent()->getSafetyMcu()->isProtectiveButton())
  {
    // Turn OFF Protective-stop signal to execute the command.
    Message::putRosConsole(TAG, 0x81400019);
    return false;
  }
  /* SafetyMCU state */
  if (this->getParent()->getSafetyMcu()->isSafeState())
  {
    // You cannot execute a command while motion preparation has not been performed.
    Message::putRosConsole(TAG, 0x81501070);
    return false;
  }
  /* other errors */
  if (this->getParent()->getDriver()->isError())
  {
    // You cannot execute a command while an error occurs.
    Message::putRosConsole(TAG, 0x81400015);
    return false;
  }
  /* brake */
  if (!this->getParent()->getBrake()->isLocked())
  {
    // Lock the brake to execute the command.
    Message::putRosConsole(TAG, 0x8140001B);
    return false;
  }

  return true;
}

/**
 * [sync] Motor on
 * @exception CobottaException An error defined by COBOTTA
 * @exception RuntimeError An other error
 */
void Motor::start(void) throw(CobottaException, std::runtime_error)
{
  /* check */
  if (this->getState() != MotorState::Off)
  {
    return;
  }
  if (!this->canStart())
  {
    // It can not execute in the current state.
    throw CobottaException(0x0F20FFFF);
  }

  /* start */
  ROS_INFO("%s: Starting...", TAG);
  this->writeHwOn(this->getParent()->getFd());
  /* sync */
  while (this->getState() != MotorState::On)
  {
    if (this->getParent()->getDriver()->isFatalError()
        || this->getParent()->getDriver()->isError())
      throw CobottaException(0x83201F83); /* Operation failed */
    if (this->getParent()->getSafetyMcu()->isSafeState())
      throw CobottaException(0x83201F83); /* Operation failed */

    ros::Duration(cobotta_common::getPeriod()).sleep();
  }
  ROS_INFO("%s: ... Motor has started.", TAG);
}

/**
 * [sync] Motor off
 * @exception CobottaException An error defined by COBOTTA
 * @exception RuntimeError An other error
 */
void Motor::stop(void) throw(CobottaException, std::runtime_error)
{
  /*
   * check
   *   except for MotorState::MOTOR_OFF to stop() brake::lock on error.
   */
  if (this->getState() == MotorState::Off
      || this->getState() == MotorState::SlowDownStop)
  {
    return;
  }
  /* off */
  ROS_INFO("%s: Stopping...", TAG);
  this->writeHwOff(this->getParent()->getFd());
  /* sync */
  while (this->getState() == MotorState::Off)
  {
    ros::Duration(cobotta_common::getPeriod()).sleep();
  }
  ROS_INFO("%s: ... Motor has stopped.", TAG);
}

/**
 * [ASYNC] Motor off
 * @exception CobottaException An error defined by COBOTTA
 * @exception RuntimeError An other error
 */
void Motor::sendStop(int fd) throw(CobottaException, std::runtime_error)
{
  ROS_INFO("%s: Stop anyway.", TAG);
  Motor::writeHwOff(fd);
}

/**
 * IOCTL_MOTOR_ON
 * @param fd File descriptor
 * @exception CobottaException An error defined by COBOTTA
 * @exception RuntimeError An other error
 */
void Motor::writeHwOn(int fd) throw(CobottaException, std::runtime_error)
{
  int ret;
  IOCTL_DATA_RESULT dat;
  int myerrno;

  memset(&dat, 0, sizeof(dat));
  errno = 0;
  ret = ioctl(fd, COBOTTA_IOCTL_MOTOR_ON, &dat);
  myerrno = errno;
  ROS_DEBUG("%s: COBOTTA_IOCTL_MOTOR_ON ret=%d errno=%d result=%lX",
            TAG, ret, myerrno, dat.result);
  if (ret) {
    ROS_ERROR("%s: ioctl(COBOTTA_IOCTL_MOTOR_ON): %s", TAG, std::strerror(myerrno));
    if (myerrno == ECANCELED) {
      throw CobottaException(dat.result);
    }
    throw std::runtime_error(std::strerror(myerrno));
  }
}

/**
 * IOCTL_MOTOR_OFF
 * @param fd File descriptor
 * @exception CobottaException An error defined by COBOTTA
 * @exception RuntimeError An other error
 */
void Motor::writeHwOff(int fd) throw(CobottaException, std::runtime_error)
{
  int ret;
  IOCTL_DATA_RESULT dat;
  int myerrno;

  memset(&dat, 0, sizeof(dat));
  errno = 0;
  ret = ioctl(fd, COBOTTA_IOCTL_MOTOR_OFF, &dat);
  myerrno = errno;
  ROS_DEBUG("%s: COBOTTA_IOCTL_MOTOR_OFF ret=%d errno=%d result=%lX",
            TAG, ret, myerrno, dat.result);
  if (ret) {
    ROS_ERROR("%s: ioctl(COBOTTA_IOCTL_MOTOR_OFF): %s", TAG, std::strerror(myerrno));
    if (myerrno == ECANCELED) {
      throw CobottaException(dat.result);
    }
    throw std::runtime_error(std::strerror(myerrno));
  }
}

/**
 * COBOTTA_IOCTL_MOTOR_STATE
 * @param fd File descriptor
 * @return 1:running 0:stop(on_proc&slowdownstop)
 * @exception CobottaException An error defined by COBOTTA
 * @exception RuntimeError An other error
 */
int Motor::readHw(int fd) throw(CobottaException, std::runtime_error)
{
  int ret;
  IOCTL_DATA_MOTOR_STATE dat;
  int myerrno;

  memset(&dat, 0, sizeof(dat));
  errno = 0;
  ret = ioctl(fd, COBOTTA_IOCTL_MOTOR_STATE, &dat);
  myerrno = errno;
  ROS_DEBUG("%s: COBOTTA_IOCTL_MOTOR_STATE ret=%d errno=%d result=%lX",
            TAG, ret, myerrno, dat.result);
  if (ret) {
    ROS_ERROR("%s: ioctl(COBOTTA_IOCTL_MOTOR_STATE): %s", TAG, std::strerror(myerrno));
    if (myerrno == ECANCELED) {
      throw CobottaException(dat.result);
    }
    throw std::runtime_error(std::strerror(myerrno));
  }
  return dat.state;
}

enum MotorState Motor::getState() const
{
  return state_;
}

std::shared_ptr<cobotta::Cobotta> Motor::getParent() const
{
  return parent_;
}
} /* namespace cobotta */
} /* namespace denso_cobotta_lib */
