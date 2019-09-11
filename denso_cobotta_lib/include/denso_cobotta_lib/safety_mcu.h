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

#ifndef SAFETY_MCU_H
#define SAFETY_MCU_H

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

enum class SafetyMcuState : int
{
  StandBy = 0,
  Normal = 1,
  SafeState = 2,
};

enum class SafetyMcuCommand : uint32_t
{
  StandBy = 0, /** safe state -> standby */
  Normal = 2, /** standby -> normal */
};

/**
 * SafetyMCU has 3 states.
 * - safe state: COBOTTA keeps a safe stop state and contains emergency-stop and protective-stop.
 * - standby: -
 * - normal: COBOTTA can start motor.
 */
class SafetyMcu
{
public:
  static constexpr const char* TAG = "SafetyMcu";

  SafetyMcu(const std::shared_ptr<Cobotta>& parent);
  SafetyMcu(const std::shared_ptr<Cobotta>& parent,
            const enum SafetyMcuState state, const int state_queue,
            const bool fatal_error, const bool error,
            const bool emergency_button, const bool protective_button);
  virtual ~SafetyMcu() = default;


  bool update(const enum SafetyMcuState state, const int state_queue,
              const bool fatal_error, const bool error,
              const bool emergency_button, const bool protective_button);

  bool canMoveState();
  void moveToStandby() throw(CobottaException, std::runtime_error);
  void moveToNormal() throw(CobottaException, std::runtime_error);
  void forceMoveToStandby() throw(CobottaException, std::runtime_error);
  struct StateCode dequeue() throw(CobottaException, std::runtime_error);

  bool isNormal() const;
  bool isSafeState() const;
  bool isEmergencyButton() const;
  bool isError() const;
  bool isFatalError() const;
  bool isProtectiveButton() const;
  enum SafetyMcuState getState() const;
  int getStateQueue() const;

private:
  static void writeHwState(const int fd,
                           const enum SafetyMcuCommand command) throw(CobottaException, std::runtime_error);
  static struct StateCode readHwQueue(const int fd) throw(CobottaException, std::runtime_error);

  std::shared_ptr<Cobotta> getParent() const;

  std::shared_ptr<Cobotta> parent_ = nullptr;
  enum SafetyMcuState state_ = SafetyMcuState::SafeState;
  int state_queue_ = 0;
  bool fatal_error_ = false;
  bool error_ = false;
  bool emergency_button_ = false;
  bool protective_button_ = false;

};

} /* namespace cobotta */
} /* namespace denso_cobotta_lib */

#endif /* SAFETY_MCU_H */
