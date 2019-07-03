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

#ifndef MESSAGE_H_
#define MESSAGE_H_

#include <string>
#include <stdint.h>
#include <unordered_map>

#include "denso_cobotta_lib/cobotta_exception.h"

namespace denso_cobotta_lib
{
namespace cobotta
{
class CobottaException;

struct MessageInfo
{
  int level;
  std::string message;
};

/**
 * Message defined by COBOTTA
 */
class Message
{
public:
  static const char* TAG;

  Message(const uint32_t code);
  Message(const uint32_t main_code, const uint32_t sub_code);
  virtual ~Message() = default;

  static struct MessageInfo getMessageInfo(const uint32_t);
  static bool isWatchdogTimerError(const uint32_t code);
  static void putRosConsole(const char* tag,
                            const uint32_t main_code, const uint32_t sub_code = 0);
  static void putRosConsole(const char* tag, const CobottaException& e);

  int getErrorLevel() const;
  uint32_t getMainCode() const;
  const std::string& getMessage() const;
  uint32_t getSubCode() const;

private:
  uint32_t main_code_;
  uint32_t sub_code_;
  int error_level_;
  std::string message_;
  static const std::unordered_map<uint32_t, struct MessageInfo> code_map_;

};

} /* namespace cobotta */
} /* namespace denso_cobotta_lib */

#endif /* MESSAGE_H_ */
