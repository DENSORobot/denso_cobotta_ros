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

#ifndef LED_H_
#define LED_H_

#include <string>
#include <stdint.h>

#include "denso_cobotta_lib/cobotta_exception.h"
#include "denso_cobotta_lib/cobotta.h"

namespace denso_cobotta_lib
{
namespace cobotta
{

class Cobotta;

struct LedColor
{
  uint8_t blink;
  uint8_t red;
  uint8_t green;
  uint8_t blue;
};

enum class LedColorTable : uint32_t
{
  White = 0x00645028,
  Red = 0x00640000,
  Green = 0xff053205,
  Blue = 0x00000032,
  Yellow = 0x005f3c00
};

class Led
{
public:
  static constexpr const char* TAG = "Led";

  Led(std::shared_ptr<Cobotta> parent);
  virtual ~Led() = default;

  void update() throw(CobottaException, std::runtime_error);

  enum LedColorTable getColorOfState();
  bool canChange();

  bool change(const uint32_t color) throw(CobottaException, std::runtime_error);
  bool change(const uint8_t blink, const uint8_t red, const uint8_t green, const uint8_t blue)
    throw(CobottaException, std::runtime_error);
  bool change(const struct LedColor color) throw(CobottaException, std::runtime_error);
  bool change(const enum LedColorTable color) throw(CobottaException, std::runtime_error);

  void forceChange(const uint32_t color) throw(CobottaException, std::runtime_error);
  void forceChange(const uint8_t blink, const uint8_t red, const uint8_t green, const uint8_t blue)
    throw(CobottaException, std::runtime_error);
  void forceChange(const struct LedColor color) throw(CobottaException, std::runtime_error);
  void forceChange(const enum LedColorTable color) throw(CobottaException, std::runtime_error);

  uint32_t receiveColor(void) throw(CobottaException, std::runtime_error);
  static struct LedColor toLedColor(const uint32_t state);
  static uint32_t toUint32(const struct LedColor color);

private:
  static void writeHw(int fd, uint32_t color) throw(CobottaException, std::runtime_error);
  static uint32_t readHw(int fd) throw(CobottaException, std::runtime_error);

  uint32_t getLastColor() const;
  void setLastColor(uint32_t color);

  std::shared_ptr<Cobotta> getParent() const;
  std::shared_ptr<Cobotta> parent_;
  /** Last setting color */
  uint32_t last_color_;

  static const bool LED_FUNCTIONAL_SAFETY;
  static const uint32_t LED_IOCTL_ARG;
};

} /* namespace cobotta */
} /* namespace denso_cobotta_lib */

#endif /* LED_H_ */
