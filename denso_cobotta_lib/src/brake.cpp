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
#include <sys/ioctl.h>

#include <ros/ros.h>

#include "denso_cobotta_lib/cobotta_common.h"
#include "denso_cobotta_lib/cobotta_ioctl.h"
#include "denso_cobotta_lib/brake.h"

namespace denso_cobotta_lib
{
namespace cobotta
{
/**
 * Constructs brake object
 * @param parent
 */
Brake::Brake(const std::shared_ptr<Cobotta>& parent)
{
  parent_ = parent;
}

/**
 * Constructs brake object
 * @param parent cobota object
 * @param array of brake state
 */
Brake::Brake(std::shared_ptr<Cobotta> parent, const std::array<std::array<int, JOINT_MAX>, ARM_MAX>& state_set)
{
  this->parent_ = parent;
  this->state_set_ = state_set;
}

/**
 * Update the current brake state
 * @param array of brake state
 * @return true:changed false:no change
 */
bool Brake::update(const std::array<std::array<int, JOINT_MAX>, ARM_MAX>& state_set)
{
  bool isChanged = false;
  bool isEqual = false;

  for(int i = 0; i < ARM_MAX; i++){
    isEqual = equal(this->state_set_[i], state_set[i]);
    if(!isEqual){
      this->state_set_[i] = state_set[i];
      isChanged = true;
    }
  }

  return isChanged;
}

/**
 * @return true all brakes are locked false: some brakes are released
 */
bool Brake::isLocked(void) const
{
  bool isLocked = true;

  for(int i = 0; i < ARM_MAX; i++){
    for(int j = 0; j < JOINT_MAX; j++){
      if(state_set_[i][j] == SRV_BRAKE_RELEASE){
        isLocked = false;
        break;
      }
    }
  }

  return isLocked;
}

/**
 * @return true:yes false:no
 */
bool Brake::canChange(void) const
{
  /*
   * On fatal error, DO NOT USE any commands
   * XXX: watch dog timer
   */
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
  /* Motor */
  if (this->getParent()->getMotor()->isRunning())
  {
    // You cannot execute a command while motor power is ON.
    Message::putRosConsole(TAG, 0x8140001D);
    return false;
  }

  return true;
}

/**
 * [sync] Change brake state
 * @param arm_no arm no
 * @param std::array update brake state
 * @exception CobottaException
 * @exception InvalidArgumentException
 */
void Brake::change(const int arm_no, const std::array<int, JOINT_MAX>& state) noexcept(false)
{
  if(arm_no < 0 || arm_no > ARM_MAX)
  {
    throw std::invalid_argument("Invalid arm_no: " + std::to_string(arm_no) +
                                " is out of range(0 <= arm_no < " + std::to_string(ARM_MAX) + ").");
  }

  if (!this->canChange())
  {
    // It can not execute in the current state.
    throw CobottaException(0x0F20FFFF);
  }

  ROS_INFO("Chaging...");
  this->writeHw(this->getParent()->getFd(), arm_no, state);

  while(this->state_set_[arm_no] != state){
    if (this->getParent()->getDriver()->isFatalError()
        || this->getParent()->getDriver()->isError())
      throw CobottaException(0x83201F83); /* Operation failed */
    if (this->getParent()->getSafetyMcu()->isSafeState())
      throw CobottaException(0x83201F83); /* Operation failed */
    if (this->getParent()->getMotor()->isRunning())
      throw CobottaException(0x83201F83); /* Operation failed */

    ros::Duration(cobotta_common::getPeriod()).sleep();
    ROS_INFO("...Change of brake has done.");
  }
}

/**
 * [sync] Release all brakes
 * @param[in] arm_no Arm number
 */
void Brake::releaseAll(const int arm_no) noexcept(false)
{
  std::array<int, JOINT_MAX> state;
  for(int i = 0; i < JOINT_MAX; i++){
    state[i] = SRV_BRAKE_RELEASE;
  }

  this->change(arm_no, state);
}

/**
 * [sync] Release all brakes
 */
void Brake::releaseAll(void) noexcept(false)
{
  std::array<int, JOINT_MAX> state;
  for(int arm_no = 0; arm_no < ARM_MAX; arm_no++){
    this->releaseAll(arm_no);
  }
}

/**
 * [sync] Lock all brakes
 * @param[in] arm_no Arm number
 */
void Brake::lockAll(const int arm_no) noexcept(false)
{
  std::array<int, JOINT_MAX> state;
  for(int i = 0; i < JOINT_MAX; i++){
    state[i] = SRV_BRAKE_LOCK;
  }

  this->change(arm_no, state);
}

/**
 * [sync] Lock all brakes
 */
void Brake::lockAll(void) noexcept(false)
{
  std::array<int, JOINT_MAX> state;
  for(int arm_no = 0; arm_no < ARM_MAX; arm_no++){
    this->lockAll(arm_no);
  }
}

/**
 * Compares  whether states of brake is equal or not
 * @param array of brake state
 * @param array of brake state
 * @return true:equal false:not equal
 */
bool Brake::equal(std::array<int, JOINT_MAX> array, std::array<int, JOINT_MAX> array1)
{
  return (array.size() == array1.size() && std::equal(array.cbegin(), array.cend(), array1.cbegin()));
}

/**
 * @param arm_no arm no
 * @return array of brake state
 */
std::array<int, JOINT_MAX> Brake::getArmState(int arm_no) noexcept(false)
{
  if(arm_no < 0 || arm_no > ARM_MAX)
  {
    throw std::invalid_argument(std::to_string(arm_no) + " is our of range (0 <= arm_no < "
                                 + std::to_string(ARM_MAX) + ").");
  }

  return state_set_[arm_no];
}

/**
 * IOCTL_SRV_SETBRAKE
 * @param fd  file descriptor
 * @param arm_no arm no
 * @param state brake state
 * @exception CobottaException An error defined by cobotta
 * @exception RuntimeError An other error
 */
void Brake::writeHw(int fd, const int arm_no, const std::array<int, JOINT_MAX>& state) noexcept(false)
{
  int ret;
  IOCTL_DATA_SETBRAKE dat;
  int myerrno;

  memset(&dat, 0, sizeof(dat));
  errno = 0;
  dat.arm_no = arm_no;
  for(int i = 0; i < JOINT_MAX; i++)
  {
    dat.brake_state[i] = state[i];
  }
  ret = ioctl(fd, COBOTTA_IOCTL_SRV_SETBRAKE, &dat);
  myerrno = errno;
  ROS_DEBUG("%s: COBOTTA_IOCTL_SRV_SETBRAKE ret=%d errno=%d result=%lX",
            TAG, ret, myerrno, dat.result);
  if(ret)
  {
    ROS_ERROR("%s: ioctl(COBOTTA_IOCTL_SRV_SETBRAKE): %s",
            TAG, std::strerror(myerrno));
    if(myerrno == ECANCELED){
      throw CobottaException(dat.result);
    }
    throw std::runtime_error(std::strerror(myerrno));
  }
}

/**
 * IOCTL_SRV_GETBRAKE
 * @param fd file descripter
 * @param arm_no arm no
 * @return array of brake states
 * @exception CobottaException An error defined by cobotta
 * @exception RuntimeError An other error
 */
std::array<int, JOINT_MAX> Brake::readHw(int fd, int arm_no) noexcept(false)
{
  int ret;
  IOCTL_DATA_GETBRAKE dat;
  int myerrno;
  std::array<int, JOINT_MAX> state;

  memset(&dat, 0, sizeof(dat));
  errno = 0;
  dat.arm_no = arm_no;
  ret = ioctl(fd, COBOTTA_IOCTL_SRV_GETBRAKE, &dat);
  myerrno = errno;
  ROS_DEBUG("%s: COBOTTA_IOCTL_SRV_GETBRAKE ret=%d errno=%d result=%lX",
            TAG, ret, myerrno, dat.result);
  if(ret)
  {
    ROS_ERROR("%s: ioctl(COBOTTA_IOCTL_GETBRAKE): %s",
            TAG, std::strerror(myerrno));
    if(myerrno == ECANCELED){
      throw CobottaException(dat.result);
    }
    throw std::runtime_error(std::strerror(myerrno));
  }

  for(int i = 0; i < JOINT_MAX; i++){
    state[i] = dat.brake_state[i];
  }

  return state;
}

std::shared_ptr<Cobotta> Brake::getParent() const
{
  return parent_;
}

std::array<std::array<int, JOINT_MAX>, ARM_MAX> Brake::getState() const
{
  return state_set_;
}

} /* namespace cobotta */
} /* namespace denso_cobotta_lib */
