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
#include <fcntl.h>
#include <sys/ioctl.h>
#include <cstring>
#include <memory>
#include <cerrno>
#include <algorithm>

#include "denso_cobotta_lib/cobotta_ioctl.h"
#include "denso_cobotta_lib/cobotta_exception.h"
#include "denso_cobotta_lib/cobotta.h"
#include "denso_cobotta_lib/gripper.h"

namespace denso_cobotta_lib
{
namespace cobotta
{
/**
 * @brief Constructs a Gripper object.
 *
 * @param Cobotta object
 */
Gripper::Gripper(const std::shared_ptr<Cobotta>& parent)
{
  parent_ = parent;
  gripper_state_ = false;
}

/**
 * Update the state of gripper.
 * @param gripper_state true:hold false:not hold
 * @exception CobottaException An error defined by cobotta
 * @exception RuntimeError An other error
 */
bool Gripper::update(bool gripper_state)
{
  bool changed = false;

  if (this->gripper_state_ != gripper_state)
  {
    this->gripper_state_ = gripper_state;
    changed = true;
  }
  return changed;
}

long Gripper::readHwGripperState(int fd) noexcept(false)
{
  IOCTL_DATA_GRIPPER_GETSTATE dat{ 0 };

  errno = 0;
  int ret = ioctl(fd, COBOTTA_IOCTL_GRIPPER_GETSTATE, &dat);
  int myerrno = errno;
  if (ret != 0)
  {
    if (myerrno == ECANCELED)
    {
      throw CobottaException(dat.result);
    }
    else
    {
      throw std::runtime_error(std::strerror(myerrno));
    }
  }
  return dat.hold;
}

/**
 * Convert string to GripperType(none/parallel/vacuum)
 * @param string
 * @return
 */
GripperType Gripper::convertGripperType(const std::string& gripper_type)
{
  if (gripper_type == "parallel")
  {
    return GripperType::Parallel;
  }
  else if (gripper_type == "vacuum")
  {
    return GripperType::Vacuum;
  }
  else if (gripper_type == "none")
  {
    return GripperType::None;
  }
  return GripperType::Undefined;
}

std::shared_ptr<Cobotta> Gripper::getParent() const
{
  return parent_;
}

} /* namespace cobotta */
} /* namespace denso_cobotta_lib */
