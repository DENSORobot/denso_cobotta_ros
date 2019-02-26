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

#ifndef _COBOTTA_STATE_H_
#define _COBOTTA_STATE_H_

#include <string>
#include <unordered_map>

#include <ros/console.h>

#include "denso_cobotta_driver/denso_cobotta_driver.h"

namespace cobotta_state
{
using denso_cobotta_driver::DensoCobottaDriver;

struct CodeValue
{
  int level;
  enum DensoCobottaDriver::LedColor led_color;
  std::string message;
};

class CobottaStateParam
{
public:
  CobottaStateParam(uint32_t main_code, uint32_t sub_code, struct CodeValue code_value);
  void putRosLog(const char* tag);
  int getLevel();
  uint64_t getLedColor();
  uint32_t getMainCode();
  uint32_t getSubCode();
  std::string getMessage();

private:
  int level_;
  enum DensoCobottaDriver::LedColor led_color_;
  uint32_t main_code_;
  uint32_t sub_code_;
  std::string message_;
 };

class CobottaState
{
public:
  static std::string getMessage(uint32_t code);
  static CobottaStateParam* getState(uint32_t main_code, uint32_t sub_code);

private:
  static std::unordered_map<uint32_t, struct CodeValue> const code_map_;
};

}  // namespace cobotta_state

#endif  // _COBOTTA_STATE_H_
