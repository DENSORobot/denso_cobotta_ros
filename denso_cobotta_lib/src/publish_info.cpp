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

#include "denso_cobotta_lib/publish_info.h"

namespace denso_cobotta_lib
{
namespace cobotta
{
/**
 *  Constructs PublishInfo object
 */
PublishInfo::PublishInfo()
{
}

/**
 *  @return the state of minus button
 */
bool PublishInfo::isMinusButton() const
{
  return minus_button_;
}

/**
 *  @param[in] the update input
 */
void PublishInfo::setMinusButton(bool minus_button)
{
  this->minus_button_ = minus_button;
}

/**
 *  @param[in] the arm no of driver queue size
 */
uint32_t PublishInfo::getDriverQueueSize(long arm_no) const noexcept(false)
{
  return driver_queue_size_[arm_no];
}

/**
 *  @return the array of driver queue size
 */
std::array<uint32_t, ARM_MAX> PublishInfo::getDriverQueueSize() const
{
  return driver_queue_size_;
}

/**
 *  @param[in] arm_no       the arm no of driver queue
 *  @param[in] queue_size   queue_size
 */
void PublishInfo::setDriverQueueSize(long arm_no, uint32_t queue_size) noexcept(false)
{
  driver_queue_size_[arm_no] = queue_size;
}

/**
 *
 */
void PublishInfo::setDriverQueueSize(const std::array<uint32_t, ARM_MAX>& driver_queue)
{
  this->driver_queue_size_ = driver_queue;
}

/**
 *  @return the state of function button
 */
bool PublishInfo::isFunctionButton() const
{
  return function_button_;
}

/**
 *  @param[in] the update input
 */
void PublishInfo::setFunctionButton(bool function_button)
{
  this->function_button_ = function_button;
}

/**
 *  @return the values of miniI/O
 */
uint16_t PublishInfo::getMiniIo() const
{
  return mini_io_;
}

/**
 *  @param[in] the update input
 */
void PublishInfo::setMiniIo(uint16_t mini_io)
{
  this->mini_io_ = mini_io;
}

/**
 *  @return the state of plus button
 */
bool PublishInfo::isPlusButton() const
{
  return plus_button_;
}

/**
 *  @param[input] the update input
 */
void PublishInfo::setPlusButton(bool plus_button)
{
  this->plus_button_ = plus_button;
}

/**
 *  @return  safety mcu queue
 */
uint32_t PublishInfo::getSafetyMcuQueueSize() const
{
  return safety_mcu_queue_size_;
}

/**
 *  @param[input] the update input
 */
void PublishInfo::setSafetyMcuQueueSize(uint32_t safety_mcu_queue_size)
{
  this->safety_mcu_queue_size_ = safety_mcu_queue_size;
}

/**
 * @return the state of gripper (grasping or not)
 */
bool PublishInfo::isGripperState() const
{
  return gripper_state_;
}

/**
 * @param[in] the state of gripper (grasping or not)
 */
void PublishInfo::setGripperState(bool gripper_state)
{
  this->gripper_state_ = gripper_state;
} 

} /* namespace cobotta */
} /* namespace denso_cobotta_lib */
