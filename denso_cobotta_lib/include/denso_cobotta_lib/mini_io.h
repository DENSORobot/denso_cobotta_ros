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

#ifndef MINI_IO_H
#define MINI_IO_H

#include <string>
#include <stdint.h>

#include "denso_cobotta_lib/cobotta_exception.h"
#include "denso_cobotta_lib/cobotta.h"

namespace denso_cobotta_lib
{
namespace cobotta
{
class Cobotta;

class MiniIo
{
public:
  static constexpr const char* TAG = "MiniIo";

  MiniIo(std::shared_ptr<Cobotta> parent);
  MiniIo(std::shared_ptr<Cobotta> parent, uint16_t input, uint16_t output);
  virtual ~MiniIo() = default;

  bool update(const uint16_t input, uint16_t output);
  void sendOutputStateValue(const uint16_t state) throw(CobottaException, std::runtime_error);
  uint16_t receiveInputStateValue() const throw(CobottaException, std::runtime_error);
  uint16_t receiveOutputStateValue() const throw(CobottaException, std::runtime_error);

private:
  static void writeHwOutput(const int fd, uint16_t value) throw(CobottaException, std::runtime_error);
  static uint16_t readHwOutput(const int fd) throw(CobottaException, std::runtime_error);
  static uint16_t readHwInput(const int fd) throw(CobottaException, std::runtime_error);

  std::shared_ptr<Cobotta> getParent() const;

  std::shared_ptr<Cobotta> parent_;
  uint16_t input_;
  uint16_t output_;

};

} /* namespace cobotta */
} /* namespace denso_cobotta_lib */

#endif /* MINI_IO_H */
