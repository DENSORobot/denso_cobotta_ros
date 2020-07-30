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
#ifndef GRIPPER_H
#define GRIPPER_H

#include "denso_cobotta_lib/cobotta.h"

namespace denso_cobotta_lib
{
namespace cobotta
{

enum class GripperType
{
  Undefined = -1,
  None = 0,
  Parallel = 1,
  Vacuum = 2,
};

class Gripper
{
public:
  static constexpr const char* TAG = "Gripper";

  Gripper(const std::shared_ptr<Cobotta>& parent);
  virtual ~Gripper() = default;

  bool update(bool gripper_state);

  static GripperType convertGripperType(const std::string& gripper_type);
  static long readHwGripperState(int fd) noexcept(false);

private:
  std::shared_ptr<Cobotta> getParent() const;
  std::shared_ptr<Cobotta> parent_;
  bool gripper_state_;
};

} /* namespace cobotta */
} /* namespace denso_cobotta_lib */
#endif /* GRIPPER_H */
