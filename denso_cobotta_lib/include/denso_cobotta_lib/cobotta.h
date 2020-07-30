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
#ifndef COBOTTA_H
#define COBOTTA_H

#include <iostream>
#include <memory>

#include "denso_cobotta_lib/publish_info.h"
#include "denso_cobotta_lib/message.h"
#include "denso_cobotta_lib/cobotta_exception.h"

#include "denso_cobotta_lib/motor.h"
#include "denso_cobotta_lib/brake.h"
#include "denso_cobotta_lib/buttons.h"
#include "denso_cobotta_lib/driver.h"
#include "denso_cobotta_lib/led.h"
#include "denso_cobotta_lib/mini_io.h"
#include "denso_cobotta_lib/safety_mcu.h"
#include "denso_cobotta_lib/gripper.h"

namespace denso_cobotta_lib
{
namespace cobotta
{
class Brake;
class Buttons;
class Driver;
class Led;
class MiniIo;
class Motor;
class SafetyMcu;
class Gripper;
class PublishInfo;

struct StateCode
{
  uint32_t main_code;
  uint32_t sub_code;
};

class Cobotta : public std::enable_shared_from_this<Cobotta>
{
public:
  static constexpr const char* TAG = "Cobotta";

  Cobotta();
  virtual ~Cobotta() = default;

  bool initialize() noexcept(false);
  void terminate();
  PublishInfo update() noexcept(false);

  int getFd() const;
  const std::unique_ptr<Brake>& getBrake() const;
  const std::unique_ptr<Buttons>& getButtons() const;
  const std::unique_ptr<Driver>& getDriver() const;
  const std::unique_ptr<Led>& getLed() const;
  const std::unique_ptr<MiniIo>& getMiniIo() const;
  const std::unique_ptr<Motor>& getMotor() const;
  const std::unique_ptr<SafetyMcu>& getSafetyMcu() const;
  const std::unique_ptr<Gripper>& getGripper() const;

private:
  int fd_;
  std::unique_ptr<Motor> motor_;
  std::unique_ptr<Brake> brake_;
  std::unique_ptr<Buttons> buttons_;
  std::unique_ptr<Driver> driver_;
  std::unique_ptr<Led> led_;
  std::unique_ptr<MiniIo> mini_io_;
  std::unique_ptr<SafetyMcu> safety_mcu_;
  std::unique_ptr<Gripper> gripper_;

  void open() noexcept(false);
  void close();
};

} /* namespace cobotta */
} /* namespace denso_cobotta_lib */

#endif /* COBOTTA_H */
