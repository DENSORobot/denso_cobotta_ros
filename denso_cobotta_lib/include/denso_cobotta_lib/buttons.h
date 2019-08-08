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

#ifndef BUTTONS_H_
#define BUTTONS_H_

#include <memory>

#include "denso_cobotta_lib/cobotta.h"

namespace denso_cobotta_lib
{
namespace cobotta
{
class Cobotta;

class Buttons
{
public:
  static constexpr const char* TAG = "Buttons";

  Buttons();
  Buttons(bool function_button, bool plus_button, bool minus_button, bool ip_reset_button);
  virtual ~Buttons() = default;

  bool update(bool function_button, bool plus_button, bool minus_button);

  bool isMinusButtonOn() const;
  bool isFunctionButtonOn() const;
  bool isIpResetButtonOn() const;
  bool isPlusButtonOn() const;

private:
  bool function_button_;
  bool plus_button_;
  bool minus_button_;
};

} /* namespace cobotta */
} /* namespace denso_cobotta_lib */

#endif /* BUTTONS_H_ */
