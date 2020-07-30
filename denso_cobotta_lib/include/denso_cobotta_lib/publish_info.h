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

#ifndef PUBLISH_INFO_H
#define PUBLISH_INFO_H

#include <stdint.h>
#include <string>

#include "denso_cobotta_lib/cobotta_ioctl.h"
#include "denso_cobotta_lib/cobotta_exception.h"

namespace denso_cobotta_lib
{
namespace cobotta
{
class PublishInfo
{
public:
  PublishInfo();
  virtual ~PublishInfo() = default;

  bool isMinusButton() const;
  void setMinusButton(bool minus_button);
  uint32_t getDriverQueueSize(long arm_no) const noexcept(false);
  std::array<uint32_t, ARM_MAX> getDriverQueueSize() const;
  void setDriverQueueSize(long arm_no, uint32_t queue_size) noexcept(false);
  void setDriverQueueSize(const std::array<uint32_t, ARM_MAX>& driver_queue);
  bool isFunctionButton() const;
  void setFunctionButton(bool function_button);
  bool isGripperState() const;
  void setGripperState(bool gripper_state);
  uint16_t getMiniIo() const;
  void setMiniIo(uint16_t miniio);
  bool isPlusButton() const;
  void setPlusButton(bool plus_button);
  uint32_t getSafetyMcuQueueSize() const;
  void setSafetyMcuQueueSize(uint32_t safety_mcu_queue_size);

private:
  std::array<uint32_t, ARM_MAX> driver_queue_size_;
  uint32_t safety_mcu_queue_size_;
  uint16_t mini_io_;
  bool function_button_;
  bool minus_button_;
  bool plus_button_;
  bool gripper_state_;
};

} /* namespace cobotta */
} /* namespace denso_cobotta_lib */

#endif /* PUBLISH_INFO_H */
