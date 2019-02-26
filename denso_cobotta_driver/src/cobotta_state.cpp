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

#include "denso_cobotta_driver/cobotta_state.h"

namespace cobotta_state
{
using denso_cobotta_driver::DensoCobottaDriver;

std::string CobottaState::getMessage(uint32_t code)
{
  struct CodeValue code_value;

  auto itr = code_map_.find(code);
  if (itr != code_map_.end())
  {
    code_value = itr->second;
    return code_value.message;
  }

  uint32_t tmp = code & 0xfffffff0;
  uint32_t axis = code & 0x0000000f;

  itr = code_map_.find(tmp);
  if (itr != code_map_.end() && axis < 9)
  {
    static char replace_str[256];
    code_value = itr->second;
    if (sprintf(replace_str, code_value.message.c_str(), axis) > 0)
    {
      return std::string(replace_str);
    }
  }
  return "Contact DENSO WAVE service.";
}

CobottaStateParam* CobottaState::getState(uint32_t main_code, uint32_t sub_code)
{
  struct CodeValue code_value;

  auto itr = code_map_.find(main_code);
  if (itr != code_map_.end())
  {
    code_value = itr-> second;
    return new CobottaStateParam(main_code, sub_code, code_value);
  }

  uint32_t tmp = main_code & 0xfffffff0;
  uint32_t axis = main_code & 0x0000000f;

  itr = code_map_.find(tmp);
  if (itr != code_map_.end() && axis < 9)
  {
    static char replace_str[256];
    code_value = itr->second;
    if (sprintf(replace_str, code_value.message.c_str(), axis) > 0)
    {
      code_value.message = replace_str;
      return new CobottaStateParam(main_code, sub_code, code_value);
    }
  }
  code_value.level = ros::console::levels::Level::Error;
  code_value.message = "Contact DENSO WAVE service.";
  return new CobottaStateParam(main_code, sub_code, code_value);
}

/*
 * class CobottaStateParam
 */
CobottaStateParam::CobottaStateParam(uint32_t main_code, uint32_t sub_code,
                                     struct CodeValue code_value)
{
  this->level_ = code_value.level;
  this->led_color_ = code_value.led_color;
  this->message_ = code_value.message;
  this->main_code_ = main_code;
  this->sub_code_ = sub_code;
}

/*
 * Print console to use ROS_[LEVEL]_STREAM.
 * <TAG>: message (main:XXXXXXXX sub:XXXXXXXXX)
 *
 */
void CobottaStateParam::putRosLog(const char* tag)
{
  switch (this->level_)
  {
    case 0:
      ROS_INFO("%s: %s (main:%08X sub:%08X)",
                      tag, this->message_.c_str(), this->main_code_, this->sub_code_);
      break;
    case 1:
      ROS_WARN("%s: %s (main:%08X sub:%08X)",
                      tag, this->message_.c_str(), this->main_code_, this->sub_code_);
      break;
    case 2:
      ROS_ERROR("%s: %s (main:%08X sub:%08X)",
                      tag, this->message_.c_str(), this->main_code_, this->sub_code_);
      break;
    case 3:
      ROS_ERROR("%s: %s (main:%08X sub:%08X)",
                      tag, this->message_.c_str(), this->main_code_, this->sub_code_);
      break;
    case 4:
      ROS_ERROR("%s: %s (main:%08X sub:%08X)",
                      tag, this->message_.c_str(), this->main_code_, this->sub_code_);
      break;
    default: // Level-5
      ROS_FATAL("%s: %s (main:%08X sub:%08X)",
                      tag, this->message_.c_str(), this->main_code_, this->sub_code_);
      break;
  }
}

int CobottaStateParam::getLevel()
{
  return this->level_;
}

uint32_t CobottaStateParam::getMainCode()
{
  return this->main_code_;
}

uint32_t CobottaStateParam::getSubCode()
{
  return this->sub_code_;
}
std::string CobottaStateParam::getMessage()
{
  return this->message_;
}

uint64_t CobottaStateParam::getLedColor()
{
  return this->led_color_;
}

std::unordered_map<uint32_t, struct CodeValue> const CobottaState::code_map_ = {
  { 0x0F200011, {0, DensoCobottaDriver::LedColor::no_change, "Emergency-stop is ON"} },
  { 0x0F200012, {0, DensoCobottaDriver::LedColor::no_change, "Emergency-stop is OFF"} },
  { 0x0F200013, {0, DensoCobottaDriver::LedColor::no_change, "Deadman switch is ON"} },
  { 0x0F200014, {0, DensoCobottaDriver::LedColor::no_change, "Deadman switch is OFF"} },
  { 0x0F200015, {0, DensoCobottaDriver::LedColor::no_change, "Protective stop is ON"} },
  { 0x0F200016, {0, DensoCobottaDriver::LedColor::no_change, "Protective stop is OFF"} },
  { 0x0F200017, {0, DensoCobottaDriver::LedColor::no_change, "Enable-auto signal is ON"} },
  { 0x0F200018, {0, DensoCobottaDriver::LedColor::no_change, "Enable-auto signal is OFF"} },
  { 0x0F200019, {0, DensoCobottaDriver::LedColor::no_change, "Changed to Manual mode"} },
  { 0x0F20001A, {0, DensoCobottaDriver::LedColor::no_change, "Changed to Auto mode"} },
  { 0x0F20001B, {0, DensoCobottaDriver::LedColor::no_change, "Power OFF(Power supply unit failure)"} },
  { 0x0F20001C, {0, DensoCobottaDriver::LedColor::no_change, "Mini I/O Output changed"} },
  { 0x0F20001D, {0, DensoCobottaDriver::LedColor::no_change, "Hand I/O Output changed"} },
  { 0x0F20001E, {0, DensoCobottaDriver::LedColor::no_change, "Synchronize the position command"} },
  { 0x0F200021, {0, DensoCobottaDriver::LedColor::no_change, "clear error(EtherCAT Motion)"} },
  { 0x0F200022, {0, DensoCobottaDriver::LedColor::no_change, "Homing attained(extended joint)"} },
  { 0x0F200031, {0, DensoCobottaDriver::LedColor::no_change, "Hand open button is ON(COBOTTA)"} },
  { 0x0F200032, {0, DensoCobottaDriver::LedColor::no_change, "Hand open button is OFF(COBOTTA)"} },
  { 0x0F200033, {0, DensoCobottaDriver::LedColor::no_change, "Hand close button is ON(COBOTTA)"} },
  { 0x0F200034, {0, DensoCobottaDriver::LedColor::no_change, "Hand close button is OFF(COBOTTA)"} },
  { 0x0F200035, {0, DensoCobottaDriver::LedColor::no_change, "Position acquition button is ON(COBOTTA)"} },
  { 0x0F200036, {0, DensoCobottaDriver::LedColor::no_change, "Position acquition button is OFF(COBOTTA)"} },
  { 0x0F200037, {0, DensoCobottaDriver::LedColor::no_change, "IP reset ON(COBOTTA)"} },
  { 0x0F200038, {0, DensoCobottaDriver::LedColor::no_change, "IP reset OFF(COBOTTA)"} },
  { 0x0F200201, {0, DensoCobottaDriver::LedColor::no_change, "Motor is ON"} },
  { 0x0F200202, {0, DensoCobottaDriver::LedColor::no_change, "Motor is OFF"} },
  { 0x0F200203, {0, DensoCobottaDriver::LedColor::no_change, "Failed to turn ON the motor"} },
  { 0x0F200204, {0, DensoCobottaDriver::LedColor::no_change, "Failed to turn OFF the motor"} },

  { 0x83201F83, {3, DensoCobottaDriver::LedColor::yellow, "Operation failed"} },

  { 0x85400001, {5, DensoCobottaDriver::LedColor::yellow, "Fatal Error"} },
  { 0x85400002, {5, DensoCobottaDriver::LedColor::yellow, "Servo module error queue overflow"} },
  { 0x85400003, {5, DensoCobottaDriver::LedColor::yellow, "Watchdog timer error"} },
  { 0x85400004, {5, DensoCobottaDriver::LedColor::yellow, "Servo module and trajectory generation module version mismatch"} },
  { 0x85400005, {5, DensoCobottaDriver::LedColor::yellow, "Servo parameter checksum error"} },
  { 0x84400006, {4, DensoCobottaDriver::LedColor::yellow, "Servo module and trajectory generation module communication error"} },
  { 0x83400007, {3, DensoCobottaDriver::LedColor::yellow, "Servo module process aborted"} },
  { 0x83400008, {3, DensoCobottaDriver::LedColor::yellow, "Servo module process timeout error"} },
  { 0x83400009, {3, DensoCobottaDriver::LedColor::yellow, "Servo module process failed."} },
  { 0x8340000A, {3, DensoCobottaDriver::LedColor::yellow, "Access denied in servo module"} },
  { 0x8340000B, {3, DensoCobottaDriver::LedColor::yellow, "There are unshown errors."} },
  { 0x8540000C, {5, DensoCobottaDriver::LedColor::yellow, "FPGA watchdog timer error."} },
  { 0x8540000D, {5, DensoCobottaDriver::LedColor::yellow, "Watchdog timer is disabled."} },
  { 0x8540000E, {5, DensoCobottaDriver::LedColor::yellow, "Communication with Safety I/O unit has stopped."} },

  { 0x85400011, {5, DensoCobottaDriver::LedColor::yellow, "You cannot execute a command while servo driver is runnin"} },
  { 0x85400012, {5, DensoCobottaDriver::LedColor::yellow, "You cannot execute a command while servo driver stops"} },
  { 0x81400013, {1, DensoCobottaDriver::LedColor::yellow, "Turn motor power OFF to execute the command."} },
  { 0x81400014, {1, DensoCobottaDriver::LedColor::yellow, "Turn motor power ON to execute the command."} },
  { 0x81400015, {1, DensoCobottaDriver::LedColor::yellow, "You cannot execute a command while an error occurs."} },
  { 0x81400016, {1, DensoCobottaDriver::LedColor::yellow, "Turn OFF Emergency-stop and execute the command clear_safe_state."} },
  { 0x81400017, {1, DensoCobottaDriver::LedColor::yellow, "Press deadman switch to execute the command."} },
  { 0x81400018, {1, DensoCobottaDriver::LedColor::yellow, "Turn ON the Enable-auto to execute the command."} },
  { 0x81400019, {1, DensoCobottaDriver::LedColor::yellow, "Turn OFF Protective-stop signal to execute the command."} },
  { 0x8140001A, {1, DensoCobottaDriver::LedColor::yellow, "You cannot execute a command while contactor is OFF"} },
  { 0x8140001B, {1, DensoCobottaDriver::LedColor::yellow, "Lock the brake to execute the command."} },
  { 0x8140001C, {1, DensoCobottaDriver::LedColor::yellow, "You cannot execute a command while robot is reducing the speed to stop."} },
  { 0x8140001D, {1, DensoCobottaDriver::LedColor::yellow, "You cannot execute a command while motor power is ON."} },
  { 0x8140001E, {1, DensoCobottaDriver::LedColor::yellow, "You cannot execute a command while motor power is OFF."} },
  { 0x81400021, {1, DensoCobottaDriver::LedColor::yellow, "Stop servo log recording to execute the command."} },
  { 0x81400022, {1, DensoCobottaDriver::LedColor::yellow, "Start servo log recording to execute the command."} },
  { 0x83400023, {3, DensoCobottaDriver::LedColor::yellow, "You cannot execute a command while one of the Emergency-stop lines is disconnected."} },
  { 0x83400024, {3, DensoCobottaDriver::LedColor::yellow, "You cannot execute a command while one of the deadman switch lines is disconnected."} },
  { 0x83400025, {3, DensoCobottaDriver::LedColor::yellow, "You cannot execute a command while one of the Enable-auto lines is disconnected."} },
  { 0x83400026, {3, DensoCobottaDriver::LedColor::yellow, "You cannot execute a command while one of the Protective-stop lines is disconnected."} },

  { 0x83400041, {3, DensoCobottaDriver::LedColor::yellow, "One of the external Emergency-stop line is disconnected. (1)"} },
  { 0x83400042, {3, DensoCobottaDriver::LedColor::yellow, "One of the external Emergency-stop line is disconnected. (2)"} },
  { 0x83400043, {3, DensoCobottaDriver::LedColor::yellow, "One of the deadman switch line is disconnected (1)"} },
  { 0x83400044, {3, DensoCobottaDriver::LedColor::yellow, "One of the deadman switch line is disconnected (2)"} },
  { 0x83400045, {3, DensoCobottaDriver::LedColor::yellow, "One of the protective switch line is disconnected (1)"} },
  { 0x83400046, {3, DensoCobottaDriver::LedColor::yellow, "One of the protective switch line is disconnected (2)"} },
  { 0x83400047, {3, DensoCobottaDriver::LedColor::yellow, "One of the Enable-auto line is not connected (1)"} },
  { 0x83400048, {3, DensoCobottaDriver::LedColor::yellow, "One of the Enable-auto line is not connected (2)"} },
  { 0x83400049, {3, DensoCobottaDriver::LedColor::yellow, "You cannot turn OFF the Emergency-stop."} },
  { 0x8340004A, {3, DensoCobottaDriver::LedColor::yellow, "You cannot turn ON deadman switch."} },
  { 0x8340004B, {3, DensoCobottaDriver::LedColor::yellow, "You cannot turn OFF Protective-stop."} },
  { 0x8340004C, {3, DensoCobottaDriver::LedColor::yellow, "You cannot turn ON Enable-auto."} },

  { 0x83400051, {3, DensoCobottaDriver::LedColor::yellow, "Incorrect entry"} },
  { 0x85400052, {5, DensoCobottaDriver::LedColor::yellow, "Invalid servo parameter"} },
  { 0x85400053, {5, DensoCobottaDriver::LedColor::yellow, "Servo parameter size is not compatible with the controller software version"} },
  { 0x85400054, {5, DensoCobottaDriver::LedColor::yellow, "Servo parameter is not assigned"} },
  { 0x83400055, {3, DensoCobottaDriver::LedColor::yellow, "Servo parameter value change error"} },
  { 0x83400056, {3, DensoCobottaDriver::LedColor::yellow, "Invalid joint"} },
  { 0x83400057, {3, DensoCobottaDriver::LedColor::yellow, "Invalid input size"} },
  { 0x83400058, {3, DensoCobottaDriver::LedColor::yellow, "Invalid output size"} },
  { 0x83400059, {3, DensoCobottaDriver::LedColor::yellow, "Invalid command"} },
  { 0x8340005A, {3, DensoCobottaDriver::LedColor::yellow, "Invalid parameter"} },
  { 0x8340005B, {3, DensoCobottaDriver::LedColor::yellow, "Invalid arm number"} },
  { 0x83404060, {3, DensoCobottaDriver::LedColor::yellow, "Joint %1$d : Invalid joint"} },

  { 0x85400071, {5, DensoCobottaDriver::LedColor::yellow, "Interrupt process delay 1"} },
  { 0x85400072, {5, DensoCobottaDriver::LedColor::yellow, "Interrupt process delay 2"} },

  { 0x8F400091, {1, DensoCobottaDriver::LedColor::yellow, "Reboot the controller"} },
  { 0x85400092, {5, DensoCobottaDriver::LedColor::yellow, "The operation cannot be executed while power supply error occurs."} },

  { 0x854000F0, {5, DensoCobottaDriver::LedColor::yellow, "Fatal error occurred."} },
  { 0x854000F1, {5, DensoCobottaDriver::LedColor::yellow, "Since the fatal error occurred, the robot controller was rebooted."} },

  { 0x83400300, {3, DensoCobottaDriver::LedColor::yellow, "I/O Failure"} },
  { 0x85400301, {5, DensoCobottaDriver::LedColor::yellow, "I/O watchdog timer error"} },
  { 0x85400310, {5, DensoCobottaDriver::LedColor::yellow, "Ch01:Mini I/O output short-circuited"} },
  { 0x85400311, {5, DensoCobottaDriver::LedColor::yellow, "Ch02:Mini I/O output short-circuited"} },
  { 0x85400312, {5, DensoCobottaDriver::LedColor::yellow, "Ch03:Mini I/O output short-circuited"} },
  { 0x85400313, {5, DensoCobottaDriver::LedColor::yellow, "Ch04:Mini I/O output short-circuited"} },
  { 0x85400314, {5, DensoCobottaDriver::LedColor::yellow, "Ch05:Mini I/O output short-circuited"} },
  { 0x85400315, {5, DensoCobottaDriver::LedColor::yellow, "Ch06:Mini I/O output short-circuited"} },
  { 0x85400316, {5, DensoCobottaDriver::LedColor::yellow, "Ch07:Mini I/O output short-circuited"} },
  { 0x85400317, {5, DensoCobottaDriver::LedColor::yellow, "Ch08:Mini I/O output short-circuited"} },
  { 0x85400318, {5, DensoCobottaDriver::LedColor::yellow, "Ch09:Mini I/O output short-circuited"} },
  { 0x85400319, {5, DensoCobottaDriver::LedColor::yellow, "Ch10:Mini I/O output short-circuited"} },
  { 0x8540031A, {5, DensoCobottaDriver::LedColor::yellow, "Ch11:Mini I/O output short-circuited"} },
  { 0x8540031B, {5, DensoCobottaDriver::LedColor::yellow, "Ch12:Mini I/O output short-circuited"} },
  { 0x8540031C, {5, DensoCobottaDriver::LedColor::yellow, "Ch13:Mini I/O output short-circuited"} },
  { 0x8540031D, {5, DensoCobottaDriver::LedColor::yellow, "Ch14:Mini I/O output short-circuited"} },
  { 0x8540031E, {5, DensoCobottaDriver::LedColor::yellow, "Ch15:Mini I/O output short-circuited"} },
  { 0x8540031F, {5, DensoCobottaDriver::LedColor::yellow, "Ch16:Mini I/O output short-circuited"} },
  { 0x85400320, {5, DensoCobottaDriver::LedColor::yellow, "Ch01:Hand I/O output short-circuited"} },
  { 0x85400321, {5, DensoCobottaDriver::LedColor::yellow, "Ch02:Hand I/O output short-circuited"} },
  { 0x85400322, {5, DensoCobottaDriver::LedColor::yellow, "Ch03:Hand I/O output short-circuited"} },
  { 0x85400323, {5, DensoCobottaDriver::LedColor::yellow, "Ch04:Hand I/O output short-circuited"} },
  { 0x85400324, {5, DensoCobottaDriver::LedColor::yellow, "Ch05:Hand I/O output short-circuited"} },
  { 0x85400325, {5, DensoCobottaDriver::LedColor::yellow, "Ch06:Hand I/O output short-circuited"} },
  { 0x85400326, {5, DensoCobottaDriver::LedColor::yellow, "Ch07:Hand I/O output short-circuited"} },
  { 0x85400327, {5, DensoCobottaDriver::LedColor::yellow, "Ch08:Hand I/O output short-circuited"} },
  { 0x85400330, {5, DensoCobottaDriver::LedColor::yellow, "I/O internal power supply 24V fuse blown"} },
  { 0x85400331, {5, DensoCobottaDriver::LedColor::yellow, "I/O internal power supply 0V fuse blown"} },
  { 0x85400332, {5, DensoCobottaDriver::LedColor::yellow, "Mini I/O power supply 24V fuse blown"} },
  { 0x85400333, {5, DensoCobottaDriver::LedColor::yellow, "Mini I/O power supply 0V fuse blown"} },
  { 0x85400334, {5, DensoCobottaDriver::LedColor::yellow, "Hand I/O power supply 24V fuse blown"} },
  { 0x85400335, {5, DensoCobottaDriver::LedColor::yellow, "Hand I/O power supply 0V fuse blown"} },
  { 0x85400336, {5, DensoCobottaDriver::LedColor::yellow, "I/O external power supply is connected in reverse polarity."} },
  { 0x85400337, {5, DensoCobottaDriver::LedColor::yellow, "I/O internal power supply is specified, but external power is supplied."} },
  { 0x8F400338, {1, DensoCobottaDriver::LedColor::yellow, "I/O external power supply is specified, but external power is not supplied."} },
  { 0x85400339, {5, DensoCobottaDriver::LedColor::yellow, "No output voltage from I/O internal power supply"} },
  { 0x8440033A, {4, DensoCobottaDriver::LedColor::yellow, "I/O external power supply is stopped."} },

  { 0x84400401, {4, DensoCobottaDriver::LedColor::yellow, "Driver unit 1 temperature error"} },
  { 0x84400402, {4, DensoCobottaDriver::LedColor::yellow, "Driver unit 2 temperature error"} },
  { 0x84400403, {4, DensoCobottaDriver::LedColor::yellow, "Driver unit 3 temperature error"} },
  { 0x84400404, {4, DensoCobottaDriver::LedColor::yellow, "Driver unit 4 temperature error"} },
  { 0x8F400409, {1, DensoCobottaDriver::LedColor::yellow, "Driver unit 1 temperature warning"} },
  { 0x8F40040A, {1, DensoCobottaDriver::LedColor::yellow, "Driver unit 2 temperature warning"} },
  { 0x8F40040B, {1, DensoCobottaDriver::LedColor::yellow, "Driver unit 3 temperature warning"} },
  { 0x8F40040C, {1, DensoCobottaDriver::LedColor::yellow, "Driver unit 4 temperature warning"} },
  { 0x84400411, {4, DensoCobottaDriver::LedColor::yellow, "Driver unit capacity has not been detected."} },
  { 0x85400412, {5, DensoCobottaDriver::LedColor::yellow, "Lower arm switch of robot brake broke down"} },
  { 0x81400413, {1, DensoCobottaDriver::LedColor::yellow, "Internal battery warning"} },
  { 0x84400414, {4, DensoCobottaDriver::LedColor::yellow, "UL lamp fuse blown"} },
  { 0x85400415, {5, DensoCobottaDriver::LedColor::yellow, "UL brake fuse blown"} },
  { 0x85400416, {5, DensoCobottaDriver::LedColor::yellow, "Robot brake broke down, but failed to query trouble point."} },
  { 0x85400417, {5, DensoCobottaDriver::LedColor::yellow, "Main robot brake lock failure"} },
  { 0x85400418, {5, DensoCobottaDriver::LedColor::yellow, "Main robot brake release failure"} },
  { 0x85400421, {5, DensoCobottaDriver::LedColor::yellow, "Fan1 stopped."} },
  { 0x85400422, {5, DensoCobottaDriver::LedColor::yellow, "Fan2 stopped."} },
  { 0x85400423, {5, DensoCobottaDriver::LedColor::yellow, "Fan3 stopped."} },
  { 0x85400424, {5, DensoCobottaDriver::LedColor::yellow, "Fan4 stopped."} },
  { 0x85400425, {5, DensoCobottaDriver::LedColor::yellow, "Fan5 stopped."} },
  { 0x85400426, {5, DensoCobottaDriver::LedColor::yellow, "Fan6 stopped."} },
  { 0x85400427, {5, DensoCobottaDriver::LedColor::yellow, "Fan7 stopped."} },
  { 0x81400431, {1, DensoCobottaDriver::LedColor::yellow, "Fan1 life span warning"} },
  { 0x81400432, {1, DensoCobottaDriver::LedColor::yellow, "Fan2 life span warning"} },
  { 0x81400433, {1, DensoCobottaDriver::LedColor::yellow, "Fan3 life span warning"} },
  { 0x81400434, {1, DensoCobottaDriver::LedColor::yellow, "Fan4 life span warning"} },
  { 0x81400435, {1, DensoCobottaDriver::LedColor::yellow, "Fan5 life span warning"} },
  { 0x81400436, {1, DensoCobottaDriver::LedColor::yellow, "Fan6 life span warning"} },
  { 0x81400437, {1, DensoCobottaDriver::LedColor::yellow, "Fan7 life span warning"} },
  { 0x85400441, {5, DensoCobottaDriver::LedColor::yellow, "Driver unit 1 board information abnormal"} },
  { 0x85400442, {5, DensoCobottaDriver::LedColor::yellow, "Driver unit 2 board information abnormal"} },
  { 0x85400443, {5, DensoCobottaDriver::LedColor::yellow, "Driver unit 3 board information abnormal"} },
  { 0x85400444, {5, DensoCobottaDriver::LedColor::yellow, "Driver unit 4 board information abnormal"} },
  { 0x84400451, {4, DensoCobottaDriver::LedColor::yellow, "Brake release by external brake release switch."} },
  { 0x84400452, {4, DensoCobottaDriver::LedColor::yellow, "Brake release by external brake release switch."} },
  { 0x83400453, {3, DensoCobottaDriver::LedColor::yellow,
                 "When brake release by external brake release switch, you cannot release brake by controller."} },
  { 0x83400454, {3, DensoCobottaDriver::LedColor::yellow, "When brake release by external brake release switch, you cannot turn motor on."} },
  { 0x83400455, {3, DensoCobottaDriver::LedColor::yellow, "External brake release switch circuit error."} },

  { 0x84404410, {4, DensoCobottaDriver::LedColor::yellow, "J%1$d:Driver unit fuse blown"} },
  { 0x84404420, {4, DensoCobottaDriver::LedColor::yellow, "J%1$d:Driver unit capacity abnormal"} },
  { 0x84404430, {4, DensoCobottaDriver::LedColor::yellow, "J%1$d:Driver unit Overcurrent (U-phase)"} },
  { 0x84404440, {4, DensoCobottaDriver::LedColor::yellow, "J%1$d:Driver unit Overcurrent (V-phase)"} },
  { 0x85404450, {5, DensoCobottaDriver::LedColor::yellow, "J%1$d:Driver unit abnormal"} },
  { 0x85404460, {5, DensoCobottaDriver::LedColor::yellow, "J%1$d:Brake fuse blown"} },
  { 0x85404470, {5, DensoCobottaDriver::LedColor::yellow, "J%1$d:Current offset abnormal (U-phase)"} },
  { 0x85404480, {5, DensoCobottaDriver::LedColor::yellow, "J%1$d:Current offset abnormal (V-phase)"} },
  { 0x85404490, {5, DensoCobottaDriver::LedColor::yellow, "J%1$d:Robot brake lock failure"} },
  { 0x854044A0, {5, DensoCobottaDriver::LedColor::yellow, "J%1$d:Robot brake broke release failure"} },
  { 0x844044B0, {4, DensoCobottaDriver::LedColor::yellow, "J%1$d:Excessive current error"} },
  { 0x844044C0, {4, DensoCobottaDriver::LedColor::yellow, "J%1$d:Excessive voltage command error"} },

  { 0x85400501, {5, DensoCobottaDriver::LedColor::yellow, "Servo calculation time over"} },
  { 0x84400502, {4, DensoCobottaDriver::LedColor::yellow, "Servo command buffer full"} },
  { 0x84400503, {4, DensoCobottaDriver::LedColor::yellow, "Command value communication delay"} },
  { 0x82400504, {2, DensoCobottaDriver::LedColor::yellow, "Servo log buffer full"} },
  { 0x82400505, {2, DensoCobottaDriver::LedColor::yellow, "DETECT buffer full"} },
  { 0x8F400506, {1, DensoCobottaDriver::LedColor::yellow, "Regeneration was not released when brake release"} },
  { 0x85400507, {5, DensoCobottaDriver::LedColor::yellow, "Initialization failed.  Reboot the robot controller."} },

  { 0x84404510, {4, DensoCobottaDriver::LedColor::yellow, "J%1$d:Excessive position error"} },
  { 0x84404520, {4, DensoCobottaDriver::LedColor::yellow, "J%1$d:Excessive position error"} },
  { 0x84404530, {4, DensoCobottaDriver::LedColor::yellow, "J%1$d:Speed command limit over"} },
  { 0x84404540, {4, DensoCobottaDriver::LedColor::yellow, "J%1$d:Acceleration command limit over"} },
  { 0x84404550, {4, DensoCobottaDriver::LedColor::yellow, "J%1$d:Motor speed limit over"} },
  { 0x84404560, {4, DensoCobottaDriver::LedColor::yellow, "J%1$d:Motor acceleration limit over"} },
  { 0x84404570, {4, DensoCobottaDriver::LedColor::yellow, "J%1$d:Motor Overcurrent"} },
  { 0x82404580, {2, DensoCobottaDriver::LedColor::yellow, "J%1$d:Arm fall detected while brake is released."} },

  { 0x84400600, {4, DensoCobottaDriver::LedColor::yellow, "Unexpected encoder error"} },
  { 0x83400601, {3, DensoCobottaDriver::LedColor::yellow, "Encoder is not initialized"} },
  { 0x83400602, {3, DensoCobottaDriver::LedColor::yellow, "Encoder initializing"} },
  { 0x83400603, {3, DensoCobottaDriver::LedColor::yellow, "Encoder error occurs"} },
  { 0x83400604, {3, DensoCobottaDriver::LedColor::yellow, "Resetting encoder"} },
  { 0x83400605, {3, DensoCobottaDriver::LedColor::yellow, "Encoder accessing"} },
  { 0x83400606, {3, DensoCobottaDriver::LedColor::yellow, "Encoder processing timeout"} },
  { 0x83400607, {3, DensoCobottaDriver::LedColor::yellow, "Invalid encoder ID"} },
  { 0x83400608, {3, DensoCobottaDriver::LedColor::yellow, "Unsupported encoder"} },
  { 0x84400609, {4, DensoCobottaDriver::LedColor::yellow, "Encoder EEPROM access failure"} },
  { 0x8440060A, {4, DensoCobottaDriver::LedColor::yellow, "Encoder EEPROM invalid address access"} },
  { 0x8440060B, {4, DensoCobottaDriver::LedColor::yellow, "Encoder EEPROM write protected area access"} },
  { 0x8540060C, {5, DensoCobottaDriver::LedColor::yellow, "Failed to specify encoder format"} },
  { 0x8540060D, {5, DensoCobottaDriver::LedColor::yellow, "This robot controller does not support the encoder"} },
  { 0x8540060E, {5, DensoCobottaDriver::LedColor::yellow, "Failed to change encoder mode"} },
  { 0x85400610, {5, DensoCobottaDriver::LedColor::yellow, "Failed to access encoder"} },
  { 0x85400611, {5, DensoCobottaDriver::LedColor::yellow, "Failed to set slave robot"} },

  { 0x844006C0, {4, DensoCobottaDriver::LedColor::yellow, "Bit stuffing error"} },
  { 0x844006C1, {4, DensoCobottaDriver::LedColor::yellow, "Bit monitor error"} },
  { 0x844006C2, {4, DensoCobottaDriver::LedColor::yellow, "Recv error"} },
  { 0x844006C3, {4, DensoCobottaDriver::LedColor::yellow, "Send error"} },

  { 0x84404600, {4, DensoCobottaDriver::LedColor::yellow, "J%1$d:Unexpected encoder error"} },
  { 0x83404610, {3, DensoCobottaDriver::LedColor::yellow, "ID%1$d:Invalid encoder ID"} },
  { 0x84404620, {4, DensoCobottaDriver::LedColor::yellow, "J%1$d:Encoder communication error"} },
  { 0x82404630, {2, DensoCobottaDriver::LedColor::yellow, "J%1$d:Encoder low battery"} },
  { 0x85404640, {5, DensoCobottaDriver::LedColor::yellow, "J%1$d:Encoder counter error1"} },
  { 0x85404650, {5, DensoCobottaDriver::LedColor::yellow, "J%1$d:Encoder counter error2"} },
  { 0x85404660, {5, DensoCobottaDriver::LedColor::yellow, "J%1$d:Encoder counter error3"} },
  { 0x85404670, {5, DensoCobottaDriver::LedColor::yellow, "J%1$d:Encoder system down"} },
  { 0x85404680, {5, DensoCobottaDriver::LedColor::yellow, "J%1$d:Encoder over speed error"} },
  { 0x85404690, {5, DensoCobottaDriver::LedColor::yellow, "J%1$d:Encoder counter overflow"} },
  { 0x834046A0, {3, DensoCobottaDriver::LedColor::yellow, "J%1$d:Encoder overheated"} },
  { 0x844046B0, {4, DensoCobottaDriver::LedColor::yellow, "J%1$d:Encoder memory data error (software)"} },
  { 0x844046C0, {4, DensoCobottaDriver::LedColor::yellow, "J%1$d:Encoder memory busy"} },
  { 0x844046D0, {4, DensoCobottaDriver::LedColor::yellow, "J%1$d:Encoder data unreceived"} },
  { 0x844046E0, {4, DensoCobottaDriver::LedColor::yellow, "J%1$d:Encoder framing error"} },
  { 0x844046F0, {4, DensoCobottaDriver::LedColor::yellow, "J%1$d:Encoder CRC check error"} },
  { 0x85404700, {5, DensoCobottaDriver::LedColor::yellow, "J%1$d:Encoder absolute error"} },
  { 0x84404710, {4, DensoCobottaDriver::LedColor::yellow, "J%1$d:Multi rotation data acquisition failure"} },
  { 0x84404720, {4, DensoCobottaDriver::LedColor::yellow, "J%1$d:Encoder communication test failure"} },
  { 0x84404730, {4, DensoCobottaDriver::LedColor::yellow, "J%1$d:Encoder temperature sensor error"} },
  { 0x84404740, {4, DensoCobottaDriver::LedColor::yellow, "J%1$d:Encoder preset error"} },
  { 0x84404750, {4, DensoCobottaDriver::LedColor::yellow, "J%1d: Unsupported encoder error 1"} },
  { 0x84404760, {4, DensoCobottaDriver::LedColor::yellow, "J%1d: Unsupported encoder error 2"} },
  { 0x84404770, {4, DensoCobottaDriver::LedColor::yellow, "J%1d: Unsupported encoder error 3"} },
  { 0x84404780, {4, DensoCobottaDriver::LedColor::yellow, "J%1d: Unsupported encoder error 4"} },
  { 0x84404790, {4, DensoCobottaDriver::LedColor::yellow, "J%1d: Unsupported encoder error 5"} },
  { 0x844047A0, {4, DensoCobottaDriver::LedColor::yellow, "J%1d: Unsupported encoder error 6"} },

  { 0x85400801, {5, DensoCobottaDriver::LedColor::yellow, "Previous backup was not completed."} },
  { 0x85400802, {5, DensoCobottaDriver::LedColor::yellow, "Data is not restored after backup failure."} },
  { 0x85400803, {5, DensoCobottaDriver::LedColor::yellow, "Storage area access failure."} },
  { 0x85400804, {5, DensoCobottaDriver::LedColor::yellow, "Backup data error."} },
  { 0x85400805, {5, DensoCobottaDriver::LedColor::yellow, "Data is not restored after backup data error."} },

  { 0x85400C00, {5, DensoCobottaDriver::LedColor::yellow, "Control MCU version mismatch"} },
  { 0x85400C01, {5, DensoCobottaDriver::LedColor::yellow, "Safety MCU version mismatch"} },
  { 0x85400C02, {5, DensoCobottaDriver::LedColor::yellow, "Reboot the robot controller to complete version up of MCU"} },
  { 0x85400C03, {5, DensoCobottaDriver::LedColor::yellow, "Failed to query control MCU about error detail"} },

  { 0x85405000, {5, DensoCobottaDriver::LedColor::yellow, "J%1$d:Data flash 1 is blank"} },
  { 0x85405010, {5, DensoCobottaDriver::LedColor::yellow, "J%1$d:Data flash 2 is blank"} },
  { 0x85405020, {5, DensoCobottaDriver::LedColor::yellow, "J%1$d:Invalid joint number"} },
  { 0x84405500, {4, DensoCobottaDriver::LedColor::yellow, "J%1$d:Bus voltage failure"} },
  { 0x84405600, {4, DensoCobottaDriver::LedColor::yellow, "J%1$d:Reveice timeout"} },
  { 0x85405C00, {5, DensoCobottaDriver::LedColor::yellow, "J%1$d:Acceleration sensor initialization error"} },
  { 0x84405F00, {4, DensoCobottaDriver::LedColor::yellow, "J%1$d:Control MCU communication error"} },
  { 0x85405FB0, {5, DensoCobottaDriver::LedColor::yellow, "J%1$d:Failed to query control MCU about error detail"} },
  { 0x85405FC0, {5, DensoCobottaDriver::LedColor::yellow, "J%1$d:Initialization error"} },
  { 0x85405FD0, {5, DensoCobottaDriver::LedColor::yellow, "J%1$d:Unexpected error"} },
  { 0x85405FE0, {5, DensoCobottaDriver::LedColor::yellow, "J%1$d:Unsupported error"} },
  { 0x84405FF0, {4, DensoCobottaDriver::LedColor::yellow, "J%1$d:Motor off"} },

  { 0x85401000, {4, DensoCobottaDriver::LedColor::yellow, "Internal error 1"} },

  { 0x85480003, {4, DensoCobottaDriver::LedColor::yellow, "Instantaneous blackout detected"} },

  { 0x854A1400, {5, DensoCobottaDriver::LedColor::red, "Motor on enable circuit error (1)"} },

  { 0x834A3757, {3, DensoCobottaDriver::LedColor::red, "Invalid parameter"} },

  { 0x854A0100, {5, DensoCobottaDriver::LedColor::red, "STO circuit error"} },
  { 0x854A0101, {5, DensoCobottaDriver::LedColor::red, "3.3V error"} },
  { 0x854A0102, {5, DensoCobottaDriver::LedColor::red, "5V error"} },
  { 0x844A0103, {4, DensoCobottaDriver::LedColor::red, "Emergency stop input circuit error"} },
  { 0x844A0104, {4, DensoCobottaDriver::LedColor::red, "Protective stop input circuit error"} },
  { 0x854A0105, {5, DensoCobottaDriver::LedColor::red, "STO state output circuit error"} },
  { 0x854A0106, {5, DensoCobottaDriver::LedColor::red, "STO state output circuit error(CPUs)"} },
  { 0x854A0107, {5, DensoCobottaDriver::LedColor::red, "AD voltage error(0V)"} },
  { 0x854A0108, {5, DensoCobottaDriver::LedColor::red, "AD voltage error(3.3V)"} },
  { 0x854A0109, {5, DensoCobottaDriver::LedColor::red, "AD converter error"} },
  { 0x854A010A, {5, DensoCobottaDriver::LedColor::red, "Safety function result mismatch"} },
  { 0x854A010B, {5, DensoCobottaDriver::LedColor::red, "sequence counter mismatch"} },
  { 0x854A010C, {5, DensoCobottaDriver::LedColor::red, "execution time mismatch"} },
  { 0x844A010D, {4, DensoCobottaDriver::LedColor::red, "emergency stop input mismatch"} },
  { 0x844A010E, {4, DensoCobottaDriver::LedColor::red, "protective stop input mismatch"} },
  { 0x854A010F, {5, DensoCobottaDriver::LedColor::red, "sequence monitor error"} },
  { 0x854A0110, {5, DensoCobottaDriver::LedColor::red, "stack diagnosis error"} },
  { 0x854A0111, {5, DensoCobottaDriver::LedColor::red, "CPU execution code diagnosis error"} },
  { 0x854A0112, {5, DensoCobottaDriver::LedColor::red, "interrupt error"} },
  { 0x854A0113, {5, DensoCobottaDriver::LedColor::red, "Monitor data ROM diagnosis error"} },
  { 0x854A0114, {5, DensoCobottaDriver::LedColor::red, "Program ROM diagnosis error"} },
  { 0x854A0115, {5, DensoCobottaDriver::LedColor::red, "Internal RAM diagnosis error"} },
  { 0x854A0116, {5, DensoCobottaDriver::LedColor::red, "Safety parameter crc error"} },
  { 0x854A0117, {5, DensoCobottaDriver::LedColor::red, "Safety parameter range error"} },
  { 0x854A0118, {5, DensoCobottaDriver::LedColor::red, "Safety parameter setting error"} },
  { 0x854A0119, {5, DensoCobottaDriver::LedColor::red, "Initialize error"} },
  { 0x854A011A, {5, DensoCobottaDriver::LedColor::red, "Internal error"} },
  { 0x854A011B, {5, DensoCobottaDriver::LedColor::red, "Communication error"} },
  { 0x854A011C, {5, DensoCobottaDriver::LedColor::red, "CAN error"} },
  { 0x854A011D, {5, DensoCobottaDriver::LedColor::red, "Timeout error"} },
  { 0x854A011E, {5, DensoCobottaDriver::LedColor::red, "sto state input mismatch"} },
  { 0x854A011F, {1, DensoCobottaDriver::LedColor::red, "watch dog timer error"} }, // XXX: Lv-1

  { 0x844A0180, {4, DensoCobottaDriver::LedColor::red, "Safety I/O monitoring device communication error (UNREC)"} },
  { 0x844A0181, {4, DensoCobottaDriver::LedColor::red, "Safety I/O monitoring device communication error (CRC)"} },
  { 0x844A0182, {4, DensoCobottaDriver::LedColor::red, "Safety I/O monitoring device communication error (FM)"} },
  { 0x844A0183, {4, DensoCobottaDriver::LedColor::red, "Safety I/O monitoring device communication error (BS)"} },
  { 0x844A0184, {4, DensoCobottaDriver::LedColor::red, "Safety I/O monitoring device communication error (BM)"} },
  { 0x844A0185, {4, DensoCobottaDriver::LedColor::red, "Safety I/O monitoring device communication error (EXT)"} },

  { 0x844A5000, {4, DensoCobottaDriver::LedColor::red, "J%1$d speed limit over"} },
  { 0x844A5010, {4, DensoCobottaDriver::LedColor::red, "J%1$d torque limit over"} },
  { 0x854A5020, {5, DensoCobottaDriver::LedColor::red, "J%1$d STO circuit error"} },
  { 0x854A5030, {5, DensoCobottaDriver::LedColor::red, "J%1$d 3.3V error"} },
  { 0x854A5040, {5, DensoCobottaDriver::LedColor::red, "J%1$d 5V error"} },
  { 0x854A5050, {5, DensoCobottaDriver::LedColor::red, "J%1$d STO state output circuit error"} },
  { 0x844A5060, {4, DensoCobottaDriver::LedColor::red, "J%1$d SLS diagnosis error"} },
  { 0x844A5070, {4, DensoCobottaDriver::LedColor::red, "J%1$d SLT diagnosis error"} },
  { 0x854A5080, {5, DensoCobottaDriver::LedColor::red, "J%1$d AD voltage error(0V)"} },
  { 0x854A5090, {5, DensoCobottaDriver::LedColor::red, "J%1$d AD voltage error(3.3V)"} },
  { 0x854A50A0, {5, DensoCobottaDriver::LedColor::red, "J%1$d AD converter error"} },
  { 0x854A50B0, {5, DensoCobottaDriver::LedColor::red, "J%1$d Safety function result mismatch"} },
  { 0x854A50C0, {5, DensoCobottaDriver::LedColor::red, "J%1$d sequence counter mismatch"} },
  { 0x854A50D0, {5, DensoCobottaDriver::LedColor::red, "J%1$d execution time mismatch"} },
  { 0x854A50E0, {5, DensoCobottaDriver::LedColor::red, "J%1$d sequence monitor error"} },
  { 0x854A50F0, {5, DensoCobottaDriver::LedColor::red, "J%1$d stack diagnosis error"} },
  { 0x854A5100, {5, DensoCobottaDriver::LedColor::red, "J%1$d CPU execution code diagnosis error"} },
  { 0x854A5110, {5, DensoCobottaDriver::LedColor::red, "J%1$d interrupt error"} },
  { 0x854A5120, {5, DensoCobottaDriver::LedColor::red, "J%1$d Monitor data ROM diagnosis error"} },
  { 0x854A5130, {5, DensoCobottaDriver::LedColor::red, "J%1$d Calibration data ROM diagnosis error"} },
  { 0x854A5140, {5, DensoCobottaDriver::LedColor::red, "J%1$d Program ROM diagnosis error"} },
  { 0x854A5150, {5, DensoCobottaDriver::LedColor::red, "J%1$d internal ROM diagnosis error"} },
  { 0x854A5160, {5, DensoCobottaDriver::LedColor::red, "J%1$d Safety parameter crc error"} },
  { 0x854A5170, {5, DensoCobottaDriver::LedColor::red, "J%1$d Safety parameter range error"} },
  { 0x854A5180, {5, DensoCobottaDriver::LedColor::red, "J%1$d Safety parameter setting error"} },
  { 0x854A5190, {5, DensoCobottaDriver::LedColor::red, "J%1$d Initialize error"} },
  { 0x854A51A0, {5, DensoCobottaDriver::LedColor::red, "J%1$d Internal error"} },
  { 0x854A51B0, {5, DensoCobottaDriver::LedColor::red, "J%1$d Communication error"} },
  { 0x854A51C0, {5, DensoCobottaDriver::LedColor::red, "J%1$d DP error"} },
  { 0x854A51D0, {5, DensoCobottaDriver::LedColor::red, "J%1$d CAN error"} },
  { 0x854A51E0, {5, DensoCobottaDriver::LedColor::red, "J%1$d Timeout error"} },
  { 0x844A51F0, {4, DensoCobottaDriver::LedColor::red, "J%1$d software limit over"} },
  { 0x854A5200, {1, DensoCobottaDriver::LedColor::red, "J%1$d watch dog timer error"} }, // XXX: Level-1
  { 0x844A5210, {4, DensoCobottaDriver::LedColor::red, "J%1$d SLS diagnosis operation not performed"} },

  { 0x844A5800, {4, DensoCobottaDriver::LedColor::red, "J%1$d Safety monitoring device communication error (UNREC)"} },
  { 0x844A5810, {4, DensoCobottaDriver::LedColor::red, "J%1$d Safety monitoring device communication error (CRC)"} },
  { 0x844A5820, {4, DensoCobottaDriver::LedColor::red, "J%1$d Safety monitoring device communication error (FM)"} },
  { 0x844A5830, {4, DensoCobottaDriver::LedColor::red, "J%1$d Safety monitoring device communication error (BS)"} },
  { 0x844A5840, {4, DensoCobottaDriver::LedColor::red, "J%1$d Safety monitoring device communication error (BM)"} },
  { 0x844A5850, {4, DensoCobottaDriver::LedColor::red, "J%1$d Safety monitoring device communication error (EXT)"} }
};

} // namespace: cobotta_state
