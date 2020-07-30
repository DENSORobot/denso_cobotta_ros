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

#include "denso_cobotta_lib/buttons.h"

namespace denso_cobotta_lib
{
namespace cobotta
{
Buttons::Buttons()
{
}

/**
 * Constructs a Button object.
 *
 *  @param[in] function_button initial function button state
 *  @param[in] plus_button     initial plus button state
 *  @param[in] minus_button    initial minus button state
 *  @param[in] ip_reset_button initial IP reset button state
 */
Buttons::Buttons(bool function_button, bool plus_button, bool minus_button, bool ip_reset_button)
{
  this->function_button_ = function_button;
  this->plus_button_ = plus_button;
  this->minus_button_ = minus_button;
}

/**
 * Update the states of buttons
 * @param[in] function_button        the update input
 * @param[in] plus_button            the update input
 * @param[in] minus_button           the update input
 * @return true:changed false:no change
 */
bool Buttons::update(bool function_button, bool plus_button, bool minus_button)
{
  bool changed = false;

  if (this->function_button_ != function_button)
  {
    this->function_button_ = function_button;
    changed = true;
  }

  if (this->plus_button_ != plus_button)
  {
    this->plus_button_ = plus_button;
    changed = true;
  }

  if (this->minus_button_ != minus_button)
  {
    this->minus_button_ = minus_button;
    changed = true;
  }

  return changed;
}

/**
 *  @return the state of minus button
 */
bool Buttons::isMinusButtonOn() const
{
  return minus_button_;
}

/**
 *  @return the state of function button
 */
bool Buttons::isFunctionButtonOn() const
{
  return function_button_;
}

/**
 *  @return the state of plus button
 */
bool Buttons::isPlusButtonOn() const
{
  return plus_button_;
}

} /* namespace cobotta */
} /* namespace denso_cobotta_lib */
