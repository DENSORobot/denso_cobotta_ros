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

#include <cerrno>
#include <sys/ioctl.h>

#include <ros/ros.h>

#include "denso_cobotta_lib/cobotta_common.h"
#include "denso_cobotta_lib/cobotta_ioctl.h"

#include "denso_cobotta_lib/led.h"

namespace denso_cobotta_lib
{
namespace cobotta
{
/**< Functional Safety [default:true] */
const bool Led::LED_FUNCTIONAL_SAFETY = true;
/**< ioctl() parameter */
const uint32_t Led::LED_IOCTL_ARG = 0x00010101;

/**
 * Constructs a LED object.
 * @param parent cobotta object
 */
Led::Led(std::shared_ptr<Cobotta> parent)
{
  this->parent_ = parent;
  this->last_color_ = 0;
}

/**
 * Update LED for functional safety.
 * green:normal red:safety yellow:error
 * @exception CobottaException An error defined by cobotta
 * @exception RuntimeError An other error
 */
void Led::update() throw(CobottaException, std::runtime_error)
{
  if (!LED_FUNCTIONAL_SAFETY)
    return;

  enum LedColorTable color = this->getColorOfState();
  if ((uint32_t)color == this->getLastColor())
    return;

  this->forceChange(this->getColorOfState());
}

/**
 * @return true:enable false:disable
 */
bool Led::canChange()
{
  if (this->getParent()->getSafetyMcu()->isFatalError()
      || this->getParent()->getDriver()->isFatalError())
  {
    // Fatal Error
    Message::putRosConsole(TAG, 0x85400001);
    return false;
  }
  /* emergency button */
  if (this->getParent()->getSafetyMcu()->isEmergencyButton())
  {
    // Turn OFF Emergency-stop and execute the command clear_safe_state.
    Message::putRosConsole(TAG, 0x81400016);
    return false;
  }
  /* protective button */
  if (this->getParent()->getSafetyMcu()->isProtectiveButton())
  {
    // Turn OFF Protective-stop signal to execute the command.
    Message::putRosConsole(TAG, 0x81400019);
    return false;
  }
  /* SafetyMCU state */
  if (this->getParent()->getSafetyMcu()->isSafeState())
  {
    // You cannot execute a command while motion preparation has not been performed.
    Message::putRosConsole(TAG, 0x81501070);
    return false;
  }
  /* other errors */
  if (this->getParent()->getDriver()->isError())
  {
    // You cannot execute a command while an error occurs.
    Message::putRosConsole(TAG, 0x81400015);
    return false;
  }
  return true;
}

/**
  * @return LED color according to the state
 */
enum LedColorTable Led::getColorOfState()
{
  /* safety state: red */
  if (this->getParent()->getSafetyMcu()->isSafeState())
  {
    return LedColorTable::Red;
  }

  /* error: yellow */
  if (this->getParent()->getDriver()->isFatalError() ||
      this->getParent()->getDriver()->isError())
  {
    return LedColorTable::Yellow;
  }
  /* normal */
  return LedColorTable::Green;
}

/**
 * Change the color of LED except for error or safe state.
 * @param color 0xCCRRGGBB
 * @return true:changed false:nochange
 * @exception CobottaException An error defined by cobotta
 * @exception RuntimeError An other error
 */
bool Led::change(const uint32_t color) throw(CobottaException, std::runtime_error)
{
  if (!LED_FUNCTIONAL_SAFETY) {
    this->forceChange(color);
    return true;
  }

  if (!this->canChange())
    return false;

  this->forceChange(color);
  return true;
}

/**
 * Change the color of LED
 * @param color 0xCCRRGGBB
 * @exception CobottaException An error defined by cobotta
 * @exception RuntimeError An other error
 */
void Led::forceChange(const uint32_t color) throw(CobottaException, std::runtime_error)
{
  std::shared_ptr<Cobotta> cb = this->getParent();
  this->writeHw(cb->getFd(), color);
  this->setLastColor(color);
}

/**
 * Change the color of LED except for error or safe state.
 * @param blink
 * @param red
 * @param green
 * @param blue
 * @return true:changed false:nochange
 * @exception CobottaException An error defined by cobotta
 * @exception RuntimeError An other error
 */
bool Led::change(const uint8_t blink, const uint8_t red, const uint8_t green,
        const uint8_t blue) throw(CobottaException, std::runtime_error)
{
  uint32_t color = Led::toUint32(LedColor{blink, red, green, blue});
  return this->change(color);
}

/**
 * Change the color of LED
 * @param blink
 * @param red
 * @param green
 * @param blue
 * @exception CobottaException An error defined by cobotta
 * @exception RuntimeError An other error
 */
void Led::forceChange(const uint8_t blink, const uint8_t red, const uint8_t green,
        const uint8_t blue) throw(CobottaException, std::runtime_error)
{
  this->forceChange(this->toUint32(LedColor{blink, red, green, blue}));
}

/**
 * Change the color of LED except for error or safe state.
 * @param color
 * @return true:changed false:nochange
 * @exception CobottaException An error defined by cobotta
 * @exception RuntimeError An other error
 */
bool Led::change(const struct LedColor color) throw(CobottaException, std::runtime_error)
{
  return this->change(this->toUint32(color));
}

/**
 * Change the color of LED
 * @param color
 * @exception CobottaException An error defined by cobotta
 * @exception RuntimeError An other error
 */
void Led::forceChange(const struct LedColor color) throw(CobottaException, std::runtime_error)
{
  this->forceChange(this->toUint32(color));
}

/**
 * Change the color of LED except for error or safe state.
 * @param color
 * @return true:changed false:nochange
 * @exception CobottaException An error defined by cobotta
 * @exception RuntimeError An other error
 */
bool Led::change(const enum LedColorTable color) throw (CobottaException, std::runtime_error)
{
  return this->change((uint32_t)color);
}
/**
 * Change the color of LED
 * @param color
 * @exception CobottaException An error defined by cobotta
 * @exception RuntimeError An other error
 */
void Led::forceChange(const enum LedColorTable color) throw (CobottaException, std::runtime_error)
{
  return this->forceChange((uint32_t)color);
}

/**
 * Ask to cobotta about LED color
 * @return 0xCCRRGGBB
 * @exception CobottaException An error defined by cobotta
 * @exception RuntimeError An other error
 */
uint32_t Led::receiveColor(void) throw(CobottaException, std::runtime_error)
{
  std::shared_ptr<Cobotta> cb = this->getParent();
  return this->readHw(cb->getFd());
}

/**
 * Convert uint32_t to struct LedColor
 * @param color 0xCCRRGGBB
 * @return
 */
struct LedColor Led::toLedColor(const uint32_t color)
{
  struct LedColor lc;
  lc.blink = (color >> 24) & 0xff;
  lc.red = (color >> 16) & 0xff;
  lc.green = (color >> 8) & 0xff;
  lc.blue = color & 0xff;

  return lc;
}

/**
 * Convert struct LedColor to uint32
 * @param color
 * @return 0xCCRRGGBB
 */
uint32_t Led::toUint32(const struct LedColor color)
{
  uint32_t value;
  value = color.blue & 0xff;
  value |= (color.green << 8) & 0xff00;
  value |= (color.red << 16) & 0xff0000;
  value |= (color.blink << 24) & 0xff000000;

  return value;
}

/**
 * @return Last setting color
 */
uint32_t Led::getLastColor() const
{
  return last_color_;
}

/**
 * @param last_color Last Setting color
 */
void Led::setLastColor(uint32_t last_color)
{
  last_color_ = last_color;
}

/**
 * [ASYNC] COBOTTA_IOCTL_SRV_PUTSTATE
 * @param fd file descriptor
 * @param color 0xCCRRGGBB
 * @exception CobottaException An error defined by cobotta
 * @exception RuntimeError An other error
 */
void Led::writeHw(const int fd, const uint32_t color) throw(CobottaException, std::runtime_error)
{
  int ret;
  IOCTL_DATA_PUTSTATE dat;
  int myerrno;

  memset(&dat, 0, sizeof(dat));
  dat.index = LED_IOCTL_ARG;
  dat.value = color;
  errno = 0;
  ret = ioctl(fd, COBOTTA_IOCTL_SRV_PUTSTATE, &dat);
  myerrno = errno;
  ROS_DEBUG("%s: COBOTTA_IOCTL_SRV_PUTSTATE, ret=%d errno=%d result=%lX",
            Led::TAG, ret, myerrno, dat.result);
  if (ret)
  {
    ROS_ERROR("%s: ioctl(COBOTTA_IOCTL_SRV_PUTSTATE): %s",
              Led::TAG, std::strerror(myerrno));
    if (myerrno == ECANCELED)
    {
      throw CobottaException(dat.result);
    }
    throw std::runtime_error(std::strerror(myerrno));
  }
}

/**
 * [ASYNC] COBOTTA_IOCTL_SRV_GETSTATE
 * @param fd file descriptor
 * @return 0xCCRRGGBB
 * @exception CobottaException An error defined by cobotta
 * @exception RuntimeError An other error
 */
uint32_t Led::readHw(const int fd) throw(CobottaException, std::runtime_error)
{
  int ret;
  IOCTL_DATA_GETSTATE dat;
  int myerrno;

  memset(&dat, 0, sizeof(dat));
  dat.index = LED_IOCTL_ARG;
  errno = 0;
  ret = ioctl(fd, COBOTTA_IOCTL_SRV_GETSTATE, &dat);
  myerrno = errno;
  ROS_DEBUG("%s: COBOTTA_IOCTL_SRV_GETSTATE ret=%d errno=%d result=%lX",
            Led::TAG, ret, myerrno, dat.result);
  if (ret)
  {
    ROS_ERROR("%s: ioctl(COBOTTA_IOCTL_SRV_GETSTATE): %s",
              TAG, std::strerror(myerrno));
    if (myerrno == ECANCELED)
    {
      throw CobottaException(dat.result);
    }
    throw std::runtime_error(std::strerror(myerrno));
  }
  return dat.value;
}

std::shared_ptr<Cobotta> Led::getParent() const
{
  return parent_;
}

} /* namespace cobotta */
} /* namespace denso_cobotta_lib */

