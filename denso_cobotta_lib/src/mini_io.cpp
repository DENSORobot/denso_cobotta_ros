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
#include "denso_cobotta_lib/cobotta_ioctl.h"
#include "denso_cobotta_lib/mini_io.h"

namespace denso_cobotta_lib
{
namespace cobotta
{
const char* MiniIo::TAG = "MiniIo";

/**
 * Constructs a MiniIo object.
 * @param parent cobotta object
 */
MiniIo::MiniIo(std::shared_ptr<Cobotta> parent)
{
  this->parent_ = parent;
}

/**
 * Constructs a MiniIo object.
 * @param parent
 * @param input
 * @param output
 */
MiniIo::MiniIo(std::shared_ptr<Cobotta> parent, uint16_t input, uint16_t output)
{
  this->parent_ = parent;
  this->input_ = input;
  this->output_ = output;
}

/**
 * Update my state.
 * @param input MiniIO input (16bit)
 * @param output MiniIO output (16bit)
 * @return true:changed false:no change
 */
bool MiniIo::update(const uint16_t input, uint16_t output)
{
  bool isChanged = false;

  if (this->input_ != input)
  {
    this->input_ = input;
    isChanged = true;
  }
  if (this->output_ != output)
  {
    this->output_ = output;
    isChanged = true;
  }
  return isChanged;
}

/**
 * [sync] Send a mini-output data to cobotta
 * @param state 16bit output value
 * @exception CobottaException An error defined by cobotta
 * @exception RuntimeError An other error
 */
void MiniIo::sendOutputStateValue(const uint16_t state) throw(CobottaException, std::runtime_error)
{
  this->writeHwOutput(this->getParent()->getFd(), state);
  while (state != this->output_)
  {
    ros::Duration(cobotta_common::getPeriod()).sleep();
  }
}

/**
 * Receive a mini-output data from cobotta
 * @return 16bit output value
 * @exception CobottaException An error defined by cobotta
 * @exception RuntimeError An other error
 */
uint16_t MiniIo::receiveOutputStateValue() const throw(CobottaException, std::runtime_error)
{
  return this->readHwOutput(this->getParent()->getFd());
}

/**
 * Receive a mini-input data from cobotta
 * @return 16bit input value
 * @exception CobottaException An error defined by cobotta
 * @exception RuntimeError An other error
 */
uint16_t MiniIo::receiveInputStateValue() const throw(CobottaException, std::runtime_error)
{
  return this->readHwInput(this->getParent()->getFd());
}

/**
 * [ASYNC] IOCTL_MINI_OUTPUT
 * @param fd file descriptor
 * @param value 16bit output value
 * @exception CobottaException An error defined by cobotta
 * @exception RuntimeError An other error
 */
void MiniIo::writeHwOutput(const int fd, uint16_t value) throw(CobottaException, std::runtime_error)
{
  int ret;
  IOCTL_DATA_MINI_OUTPUT dat;
  int myerrno;

  memset(&dat, 0, sizeof(dat));
  errno = 0;
  dat.data = value;
  dat.mask = 0xffff;
  ret = ioctl(fd, COBOTTA_IOCTL_MINI_OUTPUT, &dat);
  myerrno = errno;
  ROS_DEBUG("%s: COBOTTA_IOCTL_MINI_OUTPUT ret=%d errno=%d result=%lX\n",
            TAG, ret, myerrno, dat.result);
  if (ret)
  {
    ROS_ERROR("%s: ioctl(COBOTTA_IOCTL_MINI_OUTPUT): %s",
              TAG, std::strerror(myerrno));
    if (myerrno == ECANCELED)
    {
      throw CobottaException(dat.result);
    }
    throw std::runtime_error(std::strerror(myerrno));
  }
}
/**
 * [ASYNC] IOCTL_MINI_OUTPUT_READ
 * @param fd file descriptor
 * @return 16bit output value
 * @exception CobottaException An error defined by cobotta
 * @exception RuntimeError An other error
 */
uint16_t MiniIo::readHwOutput(const int fd) throw(CobottaException, std::runtime_error)
{
  int ret;
  IOCTL_DATA_MINI_OUTPUT_READ dat;
  int myerrno;

  memset(&dat, 0, sizeof(dat));
  errno = 0;
  ret = ioctl(fd, COBOTTA_IOCTL_MINI_OUTPUT_READ, &dat);
  myerrno = errno;
  ROS_DEBUG("%s: COBOTTA_IOCTL_MINI_OUTPUT_READ ret=%d errno=%d result=%lX\n",
            TAG, ret, myerrno, dat.result);
  if (ret)
  {
    ROS_ERROR("%s: ioctl(COBOTTA_IOCTL_MINI_OUTPUT_READ): %s", TAG, std::strerror(myerrno));
    if (myerrno == ECANCELED)
    {
      throw CobottaException(dat.result);
    }
    throw std::runtime_error(std::strerror(myerrno));
  }
  return dat.data;
}
/**
 * [ASYNC] IOCTL_MINI_INPUT
 * @param fd file descriptor
 * @return 16bit input value
 * @exception CobottaException An error defined by cobotta
 * @exception RuntimeError An other error
 */
uint16_t MiniIo::readHwInput(const int fd) throw(CobottaException, std::runtime_error)
{
  int ret;
  IOCTL_DATA_MINI_INPUT dat;
  int myerrno;

  memset(&dat, 0, sizeof(dat));
  errno = 0;
  ret = ioctl(fd, COBOTTA_IOCTL_MINI_INPUT, &dat);
  myerrno = errno;
  ROS_DEBUG("%s: COBOTTA_IOCTL_MINI_INPUT ret=%d errno=%d result=%lX\n",
            TAG, ret, myerrno, dat.result);
  if (ret)
  {
    ROS_ERROR("%s: ioctl(COBOTTA_IOCTL_MINI_INPUT): %s",
              TAG, std::strerror(myerrno));
    if (myerrno == ECANCELED)
    {
      throw CobottaException(dat.result);
    }
    throw std::runtime_error(std::strerror(myerrno));
  }
  return dat.data;
}

std::shared_ptr<Cobotta> MiniIo::getParent() const
{
  return this->parent_;
}

} /* namespace cobotta */
} /* namespace denso_cobotta_lib */
