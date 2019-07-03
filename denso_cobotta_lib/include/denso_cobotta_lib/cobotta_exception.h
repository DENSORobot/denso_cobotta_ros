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

#ifndef COBOTTA_EXCEPTION_H_
#define COBOTTA_EXCEPTION_H_

#include <exception>
#include <stdint.h>
#include <memory>

#include "denso_cobotta_lib/message.h"

namespace denso_cobotta_lib
{
namespace cobotta
{

/**
 * CobottaException
 */
class CobottaException : public std::exception
{
public:
  static const char* TAG;

  CobottaException(const uint32_t code);
  virtual ~CobottaException() = default;

  virtual const char* what() const throw();

  int getErrorLevel() const;
  uint32_t getCode() const;
  const std::string getMessage() const;
  const std::string& getWhatMessage() const;

private:
  uint32_t code_;
  int error_level_;
  std::string message_;
  std::string what_message_;
};

} /* namespace cobotta */
} /* namespace denso_cobotta_lib */

#endif /* COBOTTA_EXCEPTION_H_ */
