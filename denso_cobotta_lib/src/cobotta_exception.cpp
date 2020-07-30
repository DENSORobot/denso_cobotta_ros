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

#include <sstream>

#include <ros/ros.h>

#include "denso_cobotta_lib/cobotta_exception.h"

namespace denso_cobotta_lib
{
namespace cobotta
{
/**
 * Constructs a CobottaException object.
 * @code
 * try {
 *   throw CobottaException(0x85400001);
 *
 * } catch (CobottaException& e) {
 *   ROS_ERROR("%s", e.what());
 *   exit(1);
 * }
 * @endcode
 * @param code COBOTTA code defined by Message class
 */
CobottaException::CobottaException(const uint32_t code)
{
  struct MessageInfo info = Message::getMessageInfo(code);

  this->code_= code;
  this->message_ = info.message;
  this->error_level_ = info.level;

  std::stringstream ss;
  ss << "[Lv" << this->getErrorLevel();
  ss << ":" << std::hex << std::uppercase << std::setfill('0') << std::setw(8)<< this->getCode();
  ss << "] ";
  ss << this->getMessage();
  this->what_message_ = ss.str();
}

/**
 * Get string identifying CobottaException
 * Returns a null terminated character sequence
 * that may be used to identify the CobottaException.
 * The particular representation pointed
 * by the returned value is implementation-defined.
 * As a virtual function, derived classes may redefine this function
 * so that specific values are returned.
 *
 * @return A pointer to a c-string: "<message> [error_level:error_code]"
 */
const char* CobottaException::what() const noexcept
{
  return this->what_message_.c_str();
}

int CobottaException::getErrorLevel() const
{
  return error_level_;
}

uint32_t CobottaException::getCode() const
{
  return code_;
}

const std::string CobottaException::getMessage() const
{
  return message_;
}

const std::string& CobottaException::getWhatMessage() const
{
  return what_message_;
}

} /* namespace cobotta */
} /* namespace denso_cobotta_lib */
