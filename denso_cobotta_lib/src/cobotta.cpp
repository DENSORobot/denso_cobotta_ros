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
#include <ros/console.h>

#include "denso_cobotta_lib/cobotta_common.h"
#include "denso_cobotta_lib/cobotta.h"

namespace denso_cobotta_lib
{
namespace cobotta
{
/**
 * Constructs a Cobotta object.
 */
Cobotta::Cobotta()
{
  this->fd_ = 0;
}

/**
 * @brief Initialize
 *
 * Prepare motor_, brake_, ... .
 * Open device file.
 * Check version
 * Update all state.
 * @return true or false
 * @exception CobottaException An error defined by COBOTTA
 * @exception RuntimeError An other error
 */
bool Cobotta::initialize() throw(CobottaException, std::runtime_error)
{
  auto motor = std::make_unique<Motor>(shared_from_this());
  motor_.swap(motor);
  auto brake = std::make_unique<Brake>(shared_from_this());
  brake_.swap(brake);
  auto buttons = std::make_unique<Buttons>();
  buttons_.swap(buttons);
  auto driver = std::make_unique<Driver>(shared_from_this());
  driver_.swap(driver);
  auto led = std::make_unique<Led>(shared_from_this());
  led_.swap(led);
  auto mini_io = std::make_unique<MiniIo>(shared_from_this());
  mini_io_.swap(mini_io);
  auto safety_mcu = std::make_unique<SafetyMcu>(shared_from_this());
  safety_mcu_.swap(safety_mcu);
  auto gripper = std::make_unique<Gripper>(shared_from_this());
  gripper_.swap(gripper);

  this->close();
  this->open();
  this->getDriver()->printVersion();

  if (!getDriver()->isTargetVersion())
  {
    std::runtime_error("Invalid version of DENSO COBOTTA driver.");
    return false;
  }
  std::array<uint16_t, JOINT_MAX + 1> data;
  Driver::writeHwAcyclicCommAll(this->getFd(),
                                cobotta_common::POSITION_DEVIATION_ADDRESS | cobotta_common::ACYCLIC_WRITE_MASK,
                                cobotta_common::POSITION_DEVIATION_PARAMS, data);
  return true;
}

/**
 * @brief Terminate
 * Call me at last.
 */
void Cobotta::terminate()
{
  this->close();
}

/**
 * @brief Update all state.
 *
 * If state is changed, update LED color and motor.
 *
 * @return PublishInfo Driver and SafetyMcu state.
 * @exception CobottaException An error defined by COBOTTA
 * @exception RuntimeError An other error
 */
PublishInfo Cobotta::update() throw(CobottaException, std::runtime_error)
{
  auto all_state = getDriver()->receiveAllState();
  bool changed = false;

  changed |= getBrake()->update(all_state.brake_state);
  changed |= getButtons()->update(all_state.function_button, all_state.plus_button, all_state.minus_button);
  changed |= getDriver()->update(all_state.driver_queue_size, all_state.driver_error, all_state.driver_fatal_error,
                                 all_state.ip_reset_button);
  changed |= getMiniIo()->update(all_state.mini_io_input, all_state.mini_io_output);
  changed |= getMotor()->update((enum MotorState)all_state.motor_state);
  changed |= getSafetyMcu()->update((enum SafetyMcuState)all_state.safety_mcu_state, all_state.safety_mcu_queue_size,
                                    all_state.safety_mcu_fatal_error, all_state.safety_mcu_error,
                                    all_state.emergency_stop, all_state.protection_stop);
  changed |= getGripper()->update(all_state.gripper_state);

  if (changed)
  {
    getLed()->update();
    if (this->getMotor()->shouldStop())
    {
      Motor::sendStop(this->getFd());
    }
  }

  PublishInfo pub_info;
  pub_info.setFunctionButton(all_state.function_button);
  pub_info.setPlusButton(all_state.plus_button);
  pub_info.setMinusButton(all_state.minus_button);
  pub_info.setDriverQueueSize(all_state.driver_queue_size);
  pub_info.setSafetyMcuQueueSize(all_state.safety_mcu_queue_size);
  pub_info.setMiniIo(all_state.mini_io_input);
  pub_info.setGripperState(all_state.gripper_state);

  return pub_info;
}

const std::unique_ptr<Brake>& Cobotta::getBrake() const
{
  return brake_;
}

const std::unique_ptr<Buttons>& Cobotta::getButtons() const
{
  return buttons_;
}

const std::unique_ptr<Driver>& Cobotta::getDriver() const
{
  return driver_;
}

const std::unique_ptr<Led>& Cobotta::getLed() const
{
  return led_;
}

const std::unique_ptr<MiniIo>& Cobotta::getMiniIo() const
{
  return mini_io_;
}

const std::unique_ptr<Motor>& Cobotta::getMotor() const
{
  return motor_;
}

const std::unique_ptr<SafetyMcu>& Cobotta::getSafetyMcu() const
{
  return safety_mcu_;
}

const std::unique_ptr<Gripper>& Cobotta::getGripper() const
{
  return gripper_;
}

/**
 * Device open
 */
void Cobotta::open() throw(CobottaException)
{
  if (fd_ == 0)
  {
    fd_ = getDriver()->openDeviceFile();
  }
}

/**
 * Device close
 */
void Cobotta::close()
{
  if (fd_ != 0)
  {
    getDriver()->closeDeviceFile(fd_);
    fd_ = 0;
  }
}

int Cobotta::getFd() const
{
  return fd_;
}

} /* namespace cobotta */
} /* namespace denso_cobotta_lib */
