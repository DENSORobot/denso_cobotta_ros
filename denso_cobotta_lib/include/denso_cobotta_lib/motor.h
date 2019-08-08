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

#ifndef MOTOR_H_
#define MOTOR_H_

#include <memory>
#include <string>
#include <stdint.h>

#include "denso_cobotta_lib/cobotta_exception.h"
#include "denso_cobotta_lib/cobotta.h"

namespace denso_cobotta_lib
{
namespace cobotta
{

class Cobotta;

enum class MotorState
{
  Off = 0,
  On = 1,
  OnProc = 2,
  SlowDownStop = 3,
};

/**
 * Motor class
 * - state transition: off -> on proc -> on -> slow down stop -> off
 */
class Motor
{
public:
  static constexpr const char* TAG = "Motor";

  Motor(std::shared_ptr<Cobotta> parent);
  Motor(std::shared_ptr<Cobotta> parent, const enum MotorState state);
  virtual ~Motor() = default;

  bool update(const enum MotorState);
  bool shouldStop();
  bool isRunning(void) const;
  bool canStart(void) const;
  void start(void) throw(CobottaException, std::runtime_error);
  void stop(void) throw(CobottaException, std::runtime_error);
  static void sendStop(int fd) throw(CobottaException, std::runtime_error);

  enum MotorState getState() const;

  static int readHw(int fd) throw(CobottaException, std::runtime_error);

private:
  static void writeHwOn(int fd) throw(CobottaException, std::runtime_error);
  static void writeHwOff(int fd) throw(CobottaException, std::runtime_error);

  std::shared_ptr<cobotta::Cobotta> getParent() const;

  std::shared_ptr<cobotta::Cobotta> parent_ = nullptr;
  enum MotorState state_ = MotorState::Off;

};

} /* namespace cobotta */
} /* namespace denso_cobotta_lib */

#endif /* MOTOR_H_ */
