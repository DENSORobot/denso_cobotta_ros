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
#include <ros/ros.h>
#include "denso_cobotta_lib/message.h"

namespace denso_cobotta_lib
{
namespace cobotta
{
/**
 * Constructs a Message object.
 * @param code code number defined the message table
 */
Message::Message(const uint32_t code)
{
  struct MessageInfo info = this->getMessageInfo(code);

  this->main_code_ = code;
  this->sub_code_ = 0;
  this->error_level_ = info.level;
  this->message_ = info.message;
}

/**
 * Constructs a Message object.
 * @param main_code main_code number defined the message table
 * @param sub_code sub_code number defined the message table
 */
Message::Message(const uint32_t main_code, const uint32_t sub_code)
{
  struct MessageInfo info = this->getMessageInfo(main_code);

  this->main_code_ = main_code;
  this->sub_code_ = sub_code;
  this->error_level_ = info.level;
  this->message_ = info.message;
}

/**
 *
 * @param code code number defined the message table
 * @return message & error level
 */
struct MessageInfo Message::getMessageInfo(const uint32_t code)
{
  struct MessageInfo info = {-1, "Contact DENSO WAVE service."};

  auto itr = code_map_.find(code);
  if (itr != code_map_.end())
  {
    info = itr->second;
    return info;
  }

  uint32_t msgtype = code & 0xfffffff0;
  uint32_t axis = code & 0x0000000f;

  itr = code_map_.find(msgtype);
  if (itr != code_map_.end() && axis < 9)
  {
    static char replace_str[256];
    info = itr->second;
    if (sprintf(replace_str, info.message.c_str(), axis) > 0)
    {
      info.message = replace_str;
      return info;
    }
  }
  return info;
}

/**
 * ROS_LOG of message level.
 * @param tag
 * @param e CobottaException
 */
void Message::putRosConsole(const char* tag, const CobottaException& e)
{
  Message::putRosConsole(tag, e.getCode(), 0);
}
/**
 * ROS_LOG of message level.
 * @param tag
 * @param code
 */
void Message::putRosConsole(const char* tag,
                            const uint32_t main_code, const uint32_t sub_code)
{
  Message message = Message(main_code, sub_code);
  std::stringstream ss;

  if (tag != nullptr)
    ss << tag << ": ";

  ss << "[Lv" << message.getErrorLevel();
  ss << ":" << std::hex << std::uppercase << std::setfill('0') <<
      std::setw(8) << message.getMainCode();
  ss << "/" << std::hex << std::uppercase << std::setfill('0') <<
      std::setw(8) << message.getSubCode();
  ss << "] ";
  ss << message.getMessage();
  switch (message.getErrorLevel())
  {
    case 0:
      ROS_INFO_STREAM(ss.str());
      break;
    case 1:
      ROS_WARN_STREAM(ss.str());
      break;
    case 2:
      ROS_ERROR_STREAM(ss.str());
      break;
    case 3:
      ROS_ERROR_STREAM(ss.str());
      break;
    case 4:
      ROS_ERROR_STREAM(ss.str());
      break;
    default:  // Level-5
      ROS_FATAL_STREAM(ss.str());
      break;
  }
}

/**
 * Watch-dog-timer error 0x854A011F & 0x854A5200 should clear."
 * @param code
 * @return true: watchdog-error code false:other code
 */
bool Message::isWatchdogTimerError(const uint32_t code)
{
  if (code == 0x854A011F)
    return true;
  if ((code & 0x854A5200) == 0x854A5200)
    return true;
  // if ((code & 0x854A520F) == code)
}

int Message::getErrorLevel() const
{
  return error_level_;
}

uint32_t Message::getMainCode() const
{
  return main_code_;
}

const std::string& Message::getMessage() const
{
  return message_;
}

uint32_t Message::getSubCode() const
{
  return sub_code_;
}

const std::unordered_map<uint32_t, struct MessageInfo> Message::code_map_ = {
  { 0x0F200011, {0,  "Emergency-stop is ON"} },
  { 0x0F200012, {0,  "Emergency-stop is OFF"} },
  { 0x0F200013, {0,  "Deadman switch is ON"} },
  { 0x0F200014, {0,  "Deadman switch is OFF"} },
  { 0x0F200015, {0,  "Protective stop is ON"} },
  { 0x0F200016, {0,  "Protective stop is OFF"} },
  { 0x0F200017, {0,  "Enable-auto signal is ON"} },
  { 0x0F200018, {0,  "Enable-auto signal is OFF"} },
  { 0x0F200019, {0,  "Changed to Manual mode"} },
  { 0x0F20001A, {0,  "Changed to Auto mode"} },
  { 0x0F20001B, {0,  "Power OFF(Power supply unit failure)"} },
  { 0x0F20001C, {0,  "Mini I/O Output changed"} },
  { 0x0F20001D, {0,  "Hand I/O Output changed"} },
  { 0x0F20001E, {0,  "Synchronize the position command"} },
  { 0x0F200021, {0,  "clear error(EtherCAT Motion)"} },
  { 0x0F200022, {0,  "Homing attained(extended joint)"} },
  { 0x0F200031, {0,  "[+] button is ON(COBOTTA)"} },
  { 0x0F200032, {0,  "[+] button is OFF(COBOTTA)"} },
  { 0x0F200033, {0,  "[-] button is ON(COBOTTA)"} },
  { 0x0F200034, {0,  "[-] button is OFF(COBOTTA)"} },
  { 0x0F200035, {0,  "Function button is ON(COBOTTA)"} },
  { 0x0F200036, {0,  "Function button is OFF(COBOTTA)"} },
  { 0x0F200037, {0,  "IP reset ON(COBOTTA)"} },
  { 0x0F200038, {0,  "IP reset OFF(COBOTTA)"} },
  { 0x0F200201, {0,  "Motor is ON"} },
  { 0x0F200202, {0,  "Motor is OFF"} },
  { 0x0F200203, {0,  "Failed to turn ON the motor"} },
  { 0x0F200204, {0,  "Failed to turn OFF the motor"} },

  { 0x0F20FFFF, {0,  "It can not execute in the current state."} }, //XXX: new

  { 0x83201F83, {3,  "Operation failed"} },

  { 0x85400001, {5,  "Fatal Error. Please shutdown and start up power of COBOTTA."} },
  { 0x85400002, {5,  "Servo module error queue overflow"} },
  { 0x85400003, {5,  "Watchdog timer error"} },
  { 0x85400004, {5,  "Servo module and trajectory generation module version mismatch"} },
  { 0x85400005, {5,  "Servo parameter checksum error"} },
  { 0x84400006, {4,  "Servo module and trajectory generation module communication error"} },
  { 0x83400007, {3,  "Servo module process aborted"} },
  { 0x83400008, {3,  "Servo module process timeout error"} },
  { 0x83400009, {3,  "Servo module process failed."} },
  { 0x8340000A, {3,  "Access denied in servo module"} },
  { 0x8340000B, {3,  "There are unshown errors."} },
  { 0x8540000C, {5,  "FPGA watchdog timer error."} },
  { 0x8540000D, {5,  "Watchdog timer is disabled."} },
  { 0x8540000E, {5,  "Communication with Safety I/O unit has stopped."} },

  { 0x85400011, {5,  "You cannot execute a command while servo driver is running"} },
  { 0x85400012, {5,  "You cannot execute a command while servo driver stops"} },
  { 0x81400013, {1,  "Turn motor power OFF to execute the command."} },
  { 0x81400014, {1,  "Turn motor power ON to execute the command."} },
  { 0x81400015, {1,  "You cannot execute a command while an error occurs."} },
  { 0x81400016, {1,  "Turn OFF Emergency-stop and execute the command clear_safe_state."} },
  { 0x81400017, {1,  "Press deadman switch to execute the command."} },
  { 0x81400018, {1,  "Turn ON the Enable-auto to execute the command."} },
  { 0x81400019, {1,  "Turn OFF Protective-stop signal to execute the command."} },
  { 0x8140001A, {1,  "You cannot execute a command while contactor is OFF"} },
  { 0x8140001B, {1,  "Lock the brake to execute the command."} },
  { 0x8140001C, {1,  "You cannot execute a command while robot is reducing the speed to stop."} },
  { 0x8140001D, {1,  "You cannot execute a command while motor power is ON."} },
  { 0x8140001E, {1,  "You cannot execute a command while motor power is OFF."} },
  { 0x81400021, {1,  "Stop servo log recording to execute the command."} },
  { 0x81400022, {1,  "Start servo log recording to execute the command."} },
  { 0x83400023, {3,  "You cannot execute a command while one of the Emergency-stop lines is disconnected."} },
  { 0x83400024, {3,  "You cannot execute a command while one of the deadman switch lines is disconnected."} },
  { 0x83400025, {3,  "You cannot execute a command while one of the Enable-auto lines is disconnected."} },
  { 0x83400026, {3,  "You cannot execute a command while one of the Protective-stop lines is disconnected."} },

  { 0x83400041, {3,  "One of the external Emergency-stop line is disconnected. (1)"} },
  { 0x83400042, {3,  "One of the external Emergency-stop line is disconnected. (2)"} },
  { 0x83400043, {3,  "One of the deadman switch line is disconnected (1)"} },
  { 0x83400044, {3,  "One of the deadman switch line is disconnected (2)"} },
  { 0x83400045, {3,  "One of the protective switch line is disconnected (1)"} },
  { 0x83400046, {3,  "One of the protective switch line is disconnected (2)"} },
  { 0x83400047, {3,  "One of the Enable-auto line is not connected (1)"} },
  { 0x83400048, {3,  "One of the Enable-auto line is not connected (2)"} },
  { 0x83400049, {3,  "You cannot turn OFF the Emergency-stop."} },
  { 0x8340004A, {3,  "You cannot turn ON deadman switch."} },
  { 0x8340004B, {3,  "You cannot turn OFF Protective-stop."} },
  { 0x8340004C, {3,  "You cannot turn ON Enable-auto."} },

  { 0x83400051, {3,  "Incorrect entry"} },
  { 0x85400052, {5,  "Invalid servo parameter"} },
  { 0x85400053, {5,  "Servo parameter size is not compatible with the controller software version"} },
  { 0x85400054, {5,  "Servo parameter is not assigned"} },
  { 0x83400055, {3,  "Servo parameter value change error"} },
  { 0x83400056, {3,  "Invalid joint"} },
  { 0x83400057, {3,  "Invalid input size"} },
  { 0x83400058, {3,  "Invalid output size"} },
  { 0x83400059, {3,  "Invalid command"} },
  { 0x8340005A, {3,  "Invalid parameter"} },
  { 0x8340005B, {3,  "Invalid arm number"} },
  { 0x83404060, {3,  "Joint %1$d : Invalid joint"} },

  { 0x85400071, {5,  "Interrupt process delay 1"} },
  { 0x85400072, {5,  "Interrupt process delay 2"} },

  { 0x8F400091, {1,  "Reboot the controller"} },
  { 0x85400092, {5,  "The operation cannot be executed while power supply error occurs."} },

  { 0x854000F0, {5,  "Fatal error occurred."} },
  { 0x854000F1, {5,  "Since the fatal error occurred, the robot controller was rebooted."} },

  { 0x83400300, {3,  "I/O Failure"} },
  { 0x85400301, {5,  "I/O watchdog timer error"} },
  { 0x85400310, {5,  "Ch01:Mini I/O output short-circuited"} },
  { 0x85400311, {5,  "Ch02:Mini I/O output short-circuited"} },
  { 0x85400312, {5,  "Ch03:Mini I/O output short-circuited"} },
  { 0x85400313, {5,  "Ch04:Mini I/O output short-circuited"} },
  { 0x85400314, {5,  "Ch05:Mini I/O output short-circuited"} },
  { 0x85400315, {5,  "Ch06:Mini I/O output short-circuited"} },
  { 0x85400316, {5,  "Ch07:Mini I/O output short-circuited"} },
  { 0x85400317, {5,  "Ch08:Mini I/O output short-circuited"} },
  { 0x85400318, {5,  "Ch09:Mini I/O output short-circuited"} },
  { 0x85400319, {5,  "Ch10:Mini I/O output short-circuited"} },
  { 0x8540031A, {5,  "Ch11:Mini I/O output short-circuited"} },
  { 0x8540031B, {5,  "Ch12:Mini I/O output short-circuited"} },
  { 0x8540031C, {5,  "Ch13:Mini I/O output short-circuited"} },
  { 0x8540031D, {5,  "Ch14:Mini I/O output short-circuited"} },
  { 0x8540031E, {5,  "Ch15:Mini I/O output short-circuited"} },
  { 0x8540031F, {5,  "Ch16:Mini I/O output short-circuited"} },
  { 0x85400320, {5,  "Ch01:Hand I/O output short-circuited"} },
  { 0x85400321, {5,  "Ch02:Hand I/O output short-circuited"} },
  { 0x85400322, {5,  "Ch03:Hand I/O output short-circuited"} },
  { 0x85400323, {5,  "Ch04:Hand I/O output short-circuited"} },
  { 0x85400324, {5,  "Ch05:Hand I/O output short-circuited"} },
  { 0x85400325, {5,  "Ch06:Hand I/O output short-circuited"} },
  { 0x85400326, {5,  "Ch07:Hand I/O output short-circuited"} },
  { 0x85400327, {5,  "Ch08:Hand I/O output short-circuited"} },
  { 0x85400330, {5,  "I/O internal power supply 24V fuse blown"} },
  { 0x85400331, {5,  "I/O internal power supply 0V fuse blown"} },
  { 0x85400332, {5,  "Mini I/O power supply 24V fuse blown"} },
  { 0x85400333, {5,  "Mini I/O power supply 0V fuse blown"} },
  { 0x85400334, {5,  "Hand I/O power supply 24V fuse blown"} },
  { 0x85400335, {5,  "Hand I/O power supply 0V fuse blown"} },
  { 0x85400336, {5,  "I/O external power supply is connected in reverse polarity."} },
  { 0x85400337, {5,  "I/O internal power supply is specified, but external power is supplied."} },
  { 0x8F400338, {1,  "I/O external power supply is specified, but external power is not supplied."} },
  { 0x85400339, {5,  "No output voltage from I/O internal power supply"} },
  { 0x8440033A, {4,  "I/O external power supply is stopped."} },

  { 0x84400401, {4,  "Driver unit 1 temperature error"} },
  { 0x84400402, {4,  "Driver unit 2 temperature error"} },
  { 0x84400403, {4,  "Driver unit 3 temperature error"} },
  { 0x84400404, {4,  "Driver unit 4 temperature error"} },
  { 0x8F400409, {1,  "Driver unit 1 temperature warning"} },
  { 0x8F40040A, {1,  "Driver unit 2 temperature warning"} },
  { 0x8F40040B, {1,  "Driver unit 3 temperature warning"} },
  { 0x8F40040C, {1,  "Driver unit 4 temperature warning"} },
  { 0x84400411, {4,  "Driver unit capacity has not been detected."} },
  { 0x85400412, {5,  "Lower arm switch of robot brake broke down"} },
  { 0x81400413, {1,  "Internal battery warning"} },
  { 0x84400414, {4,  "UL lamp fuse blown"} },
  { 0x85400415, {5,  "UL brake fuse blown"} },
  { 0x85400416, {5,  "Robot brake broke down, but failed to query trouble point."} },
  { 0x85400417, {5,  "Main robot brake lock failure"} },
  { 0x85400418, {5,  "Main robot brake release failure"} },
  { 0x85400421, {5,  "Fan1 stopped."} },
  { 0x85400422, {5,  "Fan2 stopped."} },
  { 0x85400423, {5,  "Fan3 stopped."} },
  { 0x85400424, {5,  "Fan4 stopped."} },
  { 0x85400425, {5,  "Fan5 stopped."} },
  { 0x85400426, {5,  "Fan6 stopped."} },
  { 0x85400427, {5,  "Fan7 stopped."} },
  { 0x81400431, {1,  "Fan1 life span warning"} },
  { 0x81400432, {1,  "Fan2 life span warning"} },
  { 0x81400433, {1,  "Fan3 life span warning"} },
  { 0x81400434, {1,  "Fan4 life span warning"} },
  { 0x81400435, {1,  "Fan5 life span warning"} },
  { 0x81400436, {1,  "Fan6 life span warning"} },
  { 0x81400437, {1,  "Fan7 life span warning"} },
  { 0x85400441, {5,  "Driver unit 1 board information abnormal"} },
  { 0x85400442, {5,  "Driver unit 2 board information abnormal"} },
  { 0x85400443, {5,  "Driver unit 3 board information abnormal"} },
  { 0x85400444, {5,  "Driver unit 4 board information abnormal"} },
  { 0x84400451, {4,  "Brake release by external brake release switch."} },
  { 0x84400452, {4,  "Brake release by external brake release switch."} },
  { 0x83400453, {3,
                 "When brake release by external brake release switch, you cannot release brake by controller."} },
  { 0x83400454, {3,  "When brake release by external brake release switch, you cannot turn motor on."} },
  { 0x83400455, {3,  "External brake release switch circuit error."} },

  { 0x84404410, {4,  "J%1$d:Driver unit fuse blown"} },
  { 0x84404420, {4,  "J%1$d:Driver unit capacity abnormal"} },
  { 0x84404430, {4,  "J%1$d:Driver unit Overcurrent (U-phase)"} },
  { 0x84404440, {4,  "J%1$d:Driver unit Overcurrent (V-phase)"} },
  { 0x85404450, {5,  "J%1$d:Driver unit abnormal"} },
  { 0x85404460, {5,  "J%1$d:Brake fuse blown"} },
  { 0x85404470, {5,  "J%1$d:Current offset abnormal (U-phase)"} },
  { 0x85404480, {5,  "J%1$d:Current offset abnormal (V-phase)"} },
  { 0x85404490, {5,  "J%1$d:Robot brake lock failure"} },
  { 0x854044A0, {5,  "J%1$d:Robot brake broke release failure"} },
  { 0x844044B0, {4,  "J%1$d:Excessive current error"} },
  { 0x844044C0, {4,  "J%1$d:Excessive voltage command error"} },

  { 0x85400501, {5,  "Servo calculation time over"} },
  { 0x84400502, {4,  "Servo command buffer full"} },
  { 0x84400503, {4,  "Command value communication delay"} },
  { 0x82400504, {2,  "Servo log buffer full"} },
  { 0x82400505, {2,  "DETECT buffer full"} },
  { 0x8F400506, {1,  "Regeneration was not released when brake release"} },
  { 0x85400507, {5,  "Initialization failed.  Reboot the robot controller."} },

  { 0x84404510, {4,  "J%1$d:Excessive position error"} },
  { 0x84404520, {4,  "J%1$d:Excessive position error"} },
  { 0x84404530, {4,  "J%1$d:Speed command limit over"} },
  { 0x84404540, {4,  "J%1$d:Acceleration command limit over"} },
  { 0x84404550, {4,  "J%1$d:Motor speed limit over"} },
  { 0x84404560, {4,  "J%1$d:Motor acceleration limit over"} },
  { 0x84404570, {4,  "J%1$d:Motor Overcurrent"} },
  { 0x82404580, {2,  "J%1$d:Arm fall detected while brake is released."} },

  { 0x84400600, {4,  "Unexpected encoder error"} },
  { 0x83400601, {3,  "Encoder is not initialized"} },
  { 0x83400602, {3,  "Encoder initializing"} },
  { 0x83400603, {3,  "Encoder error occurs"} },
  { 0x83400604, {3,  "Resetting encoder"} },
  { 0x83400605, {3,  "Encoder accessing"} },
  { 0x83400606, {3,  "Encoder processing timeout"} },
  { 0x83400607, {3,  "Invalid encoder ID"} },
  { 0x83400608, {3,  "Unsupported encoder"} },
  { 0x84400609, {4,  "Encoder EEPROM access failure"} },
  { 0x8440060A, {4,  "Encoder EEPROM invalid address access"} },
  { 0x8440060B, {4,  "Encoder EEPROM write protected area access"} },
  { 0x8540060C, {5,  "Failed to specify encoder format"} },
  { 0x8540060D, {5,  "This robot controller does not support the encoder"} },
  { 0x8540060E, {5,  "Failed to change encoder mode"} },
  { 0x85400610, {5,  "Failed to access encoder"} },
  { 0x85400611, {5,  "Failed to set slave robot"} },

  { 0x844006C0, {4,  "Bit stuffing error"} },
  { 0x844006C1, {4,  "Bit monitor error"} },
  { 0x844006C2, {4,  "Recv error"} },
  { 0x844006C3, {4,  "Send error"} },

  { 0x84404600, {4,  "J%1$d:Unexpected encoder error"} },
  { 0x83404610, {3,  "ID%1$d:Invalid encoder ID"} },
  { 0x84404620, {4,  "J%1$d:Encoder communication error"} },
  { 0x82404630, {2,  "J%1$d:Encoder low battery"} },
  { 0x85404640, {5,  "J%1$d:Encoder counter error1"} },
  { 0x85404650, {5,  "J%1$d:Encoder counter error2"} },
  { 0x85404660, {5,  "J%1$d:Encoder counter error3"} },
  { 0x85404670, {5,  "J%1$d:Encoder system down"} },
  { 0x85404680, {5,  "J%1$d:Encoder over speed error"} },
  { 0x85404690, {5,  "J%1$d:Encoder counter overflow"} },
  { 0x834046A0, {3,  "J%1$d:Encoder overheated"} },
  { 0x844046B0, {4,  "J%1$d:Encoder memory data error (software)"} },
  { 0x844046C0, {4,  "J%1$d:Encoder memory busy"} },
  { 0x844046D0, {4,  "J%1$d:Encoder data unreceived"} },
  { 0x844046E0, {4,  "J%1$d:Encoder framing error"} },
  { 0x844046F0, {4,  "J%1$d:Encoder CRC check error"} },
  { 0x85404700, {5,  "J%1$d:Encoder absolute error"} },
  { 0x84404710, {4,  "J%1$d:Multi rotation data acquisition failure"} },
  { 0x84404720, {4,  "J%1$d:Encoder communication test failure"} },
  { 0x84404730, {4,  "J%1$d:Encoder temperature sensor error"} },
  { 0x84404740, {4,  "J%1$d:Encoder preset error"} },
  { 0x84404750, {4,  "J%1d: Unsupported encoder error 1"} },
  { 0x84404760, {4,  "J%1d: Unsupported encoder error 2"} },
  { 0x84404770, {4,  "J%1d: Unsupported encoder error 3"} },
  { 0x84404780, {4,  "J%1d: Unsupported encoder error 4"} },
  { 0x84404790, {4,  "J%1d: Unsupported encoder error 5"} },
  { 0x844047A0, {4,  "J%1d: Unsupported encoder error 6"} },

  { 0x85400801, {5,  "Previous backup was not completed."} },
  { 0x85400802, {5,  "Data is not restored after backup failure."} },
  { 0x85400803, {5,  "Storage area access failure."} },
  { 0x85400804, {5,  "Backup data error."} },
  { 0x85400805, {5,  "Data is not restored after backup data error."} },

  { 0x85400C00, {5,  "Control MCU version mismatch"} },
  { 0x85400C01, {5,  "Safety MCU version mismatch"} },
  { 0x85400C02, {5,  "Reboot the robot controller to complete version up of MCU"} },
  { 0x85400C03, {5,  "Failed to query control MCU about error detail"} },

  { 0x85405000, {5,  "J%1$d:Data flash 1 is blank"} },
  { 0x85405010, {5,  "J%1$d:Data flash 2 is blank"} },
  { 0x85405020, {5,  "J%1$d:Invalid joint number"} },
  { 0x84405500, {4,  "J%1$d:Bus voltage failure"} },
  { 0x84405600, {4,  "J%1$d:Reveice timeout"} },
  { 0x85405C00, {5,  "J%1$d:Acceleration sensor initialization error"} },
  { 0x84405F00, {4,  "J%1$d:Control MCU communication error"} },
  { 0x85405FB0, {5,  "J%1$d:Failed to query control MCU about error detail"} },
  { 0x85405FC0, {5,  "J%1$d:Initialization error"} },
  { 0x85405FD0, {5,  "J%1$d:Unexpected error"} },
  { 0x85405FE0, {5,  "J%1$d:Unsupported error"} },
  { 0x84405FF0, {4,  "J%1$d:Motor off"} },

  { 0x85401000, {4,  "Internal error 1"} },

  { 0x85480003, {4,  "Instantaneous blackout detected"} },

  { 0x854A1400, {5,  "Motor on enable circuit error (1)"} },

  { 0x834A3757, {3,  "Invalid parameter"} },

  { 0x854A0100, {5,  "STO circuit error"} },
  { 0x854A0101, {5,  "3.3V error"} },
  { 0x854A0102, {5,  "5V error"} },
  { 0x844A0103, {4,  "Emergency stop input circuit error"} },
  { 0x844A0104, {4,  "Protective stop input circuit error"} },
  { 0x854A0105, {5,  "STO state output circuit error"} },
  { 0x854A0106, {5,  "STO state output circuit error(CPUs)"} },
  { 0x854A0107, {5,  "AD voltage error(0V)"} },
  { 0x854A0108, {5,  "AD voltage error(3.3V)"} },
  { 0x854A0109, {5,  "AD converter error"} },
  { 0x854A010A, {5,  "Safety function result mismatch"} },
  { 0x854A010B, {5,  "sequence counter mismatch"} },
  { 0x854A010C, {5,  "execution time mismatch"} },
  { 0x844A010D, {4,  "emergency stop input mismatch"} },
  { 0x844A010E, {4,  "protective stop input mismatch"} },
  { 0x854A010F, {5,  "sequence monitor error"} },
  { 0x854A0110, {5,  "stack diagnosis error"} },
  { 0x854A0111, {5,  "CPU execution code diagnosis error"} },
  { 0x854A0112, {5,  "interrupt error"} },
  { 0x854A0113, {5,  "Monitor data ROM diagnosis error"} },
  { 0x854A0114, {5,  "Program ROM diagnosis error"} },
  { 0x854A0115, {5,  "Internal RAM diagnosis error"} },
  { 0x854A0116, {5,  "Safety parameter crc error"} },
  { 0x854A0117, {5,  "Safety parameter range error"} },
  { 0x854A0118, {5,  "Safety parameter setting error"} },
  { 0x854A0119, {5,  "Initialize error"} },
  { 0x854A011A, {5,  "Internal error"} },
  { 0x854A011B, {5,  "Communication error"} },
  { 0x854A011C, {5,  "CAN error"} },
  { 0x854A011D, {5,  "Timeout error"} },
  { 0x854A011E, {5,  "sto state input mismatch"} },
  { 0x854A011F, {1,  "watch dog timer error"} }, /* Lv1 OSS only */

  { 0x844A0180, {4,  "Safety I/O monitoring device communication error (UNREC)"} },
  { 0x844A0181, {4,  "Safety I/O monitoring device communication error (CRC)"} },
  { 0x844A0182, {4,  "Safety I/O monitoring device communication error (FM)"} },
  { 0x844A0183, {4,  "Safety I/O monitoring device communication error (BS)"} },
  { 0x844A0184, {4,  "Safety I/O monitoring device communication error (BM)"} },
  { 0x844A0185, {4,  "Safety I/O monitoring device communication error (EXT)"} },

  { 0x844A5000, {4,  "J%1$d speed limit over"} },
  { 0x844A5010, {4,  "J%1$d torque limit over"} },
  { 0x854A5020, {5,  "J%1$d STO circuit error"} },
  { 0x854A5030, {5,  "J%1$d 3.3V error"} },
  { 0x854A5040, {5,  "J%1$d 5V error"} },
  { 0x854A5050, {5,  "J%1$d STO state output circuit error"} },
  { 0x844A5060, {4,  "J%1$d SLS diagnosis error"} },
  { 0x844A5070, {4,  "J%1$d SLT diagnosis error"} },
  { 0x854A5080, {5,  "J%1$d AD voltage error(0V)"} },
  { 0x854A5090, {5,  "J%1$d AD voltage error(3.3V)"} },
  { 0x854A50A0, {5,  "J%1$d AD converter error"} },
  { 0x854A50B0, {5,  "J%1$d Safety function result mismatch"} },
  { 0x854A50C0, {5,  "J%1$d sequence counter mismatch"} },
  { 0x854A50D0, {5,  "J%1$d execution time mismatch"} },
  { 0x854A50E0, {5,  "J%1$d sequence monitor error"} },
  { 0x854A50F0, {5,  "J%1$d stack diagnosis error"} },
  { 0x854A5100, {5,  "J%1$d CPU execution code diagnosis error"} },
  { 0x854A5110, {5,  "J%1$d interrupt error"} },
  { 0x854A5120, {5,  "J%1$d Monitor data ROM diagnosis error"} },
  { 0x854A5130, {5,  "J%1$d Calibration data ROM diagnosis error"} },
  { 0x854A5140, {5,  "J%1$d Program ROM diagnosis error"} },
  { 0x854A5150, {5,  "J%1$d internal ROM diagnosis error"} },
  { 0x854A5160, {5,  "J%1$d Safety parameter crc error"} },
  { 0x854A5170, {5,  "J%1$d Safety parameter range error"} },
  { 0x854A5180, {5,  "J%1$d Safety parameter setting error"} },
  { 0x854A5190, {5,  "J%1$d Initialize error"} },
  { 0x854A51A0, {5,  "J%1$d Internal error"} },
  { 0x854A51B0, {5,  "J%1$d Communication error"} },
  { 0x854A51C0, {5,  "J%1$d DP error"} },
  { 0x854A51D0, {5,  "J%1$d CAN error"} },
  { 0x854A51E0, {5,  "J%1$d Timeout error"} },
  { 0x844A51F0, {4,  "J%1$d software limit over"} },
  { 0x854A5200, {1,  "J%1$d watch dog timer error"} }, /* Lv1 OSS only */
  { 0x844A5210, {4,  "J%1$d SLS diagnosis operation not performed"} },

  { 0x844A5800, {4,  "J%1$d Safety monitoring device communication error (UNREC)"} },
  { 0x844A5810, {4,  "J%1$d Safety monitoring device communication error (CRC)"} },
  { 0x844A5820, {4,  "J%1$d Safety monitoring device communication error (FM)"} },
  { 0x844A5830, {4,  "J%1$d Safety monitoring device communication error (BS)"} },
  { 0x844A5840, {4,  "J%1$d Safety monitoring device communication error (BM)"} },
  { 0x844A5850, {4,  "J%1$d Safety monitoring device communication error (EXT)"} },

  { 0x81501070 ,{1,  "You cannot execute a command while motion preparation has not been performed."}}

};

} /* namespace cobotta */
} /* namespace denso_cobotta_lib */
