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


#ifndef BRAKE_H
#define BRAKE_H

#include <iostream>
#include <memory>
#include <stdint.h>

#include "denso_cobotta_lib/cobotta_exception.h"
#include "denso_cobotta_lib/cobotta.h"

namespace denso_cobotta_lib
{
namespace cobotta
{
class Cobotta;

/**
 * Brake class
 */
class Brake
{
public:
  static constexpr const char* TAG = "Brake";

  Brake(const std::shared_ptr<Cobotta>& parent);
  Brake(std::shared_ptr<Cobotta> parent, const std::array<std::array<int, JOINT_MAX>, ARM_MAX>& state_set);
  virtual ~Brake() = default;

  bool update(const std::array<std::array<int, JOINT_MAX>, ARM_MAX>& state_set);
  bool isLocked(void) const;
  bool canChange(void) const;
  void change(const int arm_no, const std::array<int, JOINT_MAX>& state) noexcept(false);
  void releaseAll(const int arm_no) noexcept(false);
  void releaseAll(void) noexcept(false);
  void lockAll(const int arm_no) noexcept(false);
  void lockAll(void) noexcept(false);
  bool equal(std::array<int, JOINT_MAX>, std::array<int, JOINT_MAX>);
  std::array<int, JOINT_MAX> getArmState(int arm_no) noexcept(false);

  std::array<std::array<int, JOINT_MAX>, ARM_MAX> getState() const;

private:
  static void writeHw(int fd, const int arm_no, const std::array<int, JOINT_MAX>& state) noexcept(false);
  static std::array<int, JOINT_MAX> readHw(int fd, int arm_no) noexcept(false);

  std::shared_ptr<Cobotta> getParent() const;

  std::shared_ptr<Cobotta> parent_;
  std::array<std::array<int, JOINT_MAX>, ARM_MAX> state_set_;
};

} /* namespace cobotta */
} /* namespace denso_cobotta_lib */

#endif /* BRAKE_H */
