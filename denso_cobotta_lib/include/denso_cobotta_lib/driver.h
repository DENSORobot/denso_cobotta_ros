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

#ifndef DRIVER_H_
#define DRIVER_H_

#include <memory>
#include <array>
#include <string>
#include <stdint.h>

#include "denso_cobotta_lib/cobotta_exception.h"
#include "denso_cobotta_lib/cobotta.h"

namespace denso_cobotta_lib
{
namespace cobotta
{
class Cobotta;
struct StateCode;

/**
 * IOCTL_GET_ALLSTATE
 */
struct DriverStateInfo
{
  std::array<uint32_t, ARM_MAX> driver_queue_size;
  uint32_t safety_mcu_queue_size;
  int motor_state;
  int safety_mcu_state;
  std::array<std::array<int, JOINT_MAX>, ARM_MAX> brake_state;
  bool emergency_stop;
  bool protection_stop;
  bool function_button;
  bool plus_button;
  bool minus_button;
  bool ip_reset_button;
  bool gripper_state;
  bool driver_error;
  bool driver_fatal_error;
  bool safety_mcu_error;
  bool safety_mcu_fatal_error;
  uint16_t mini_io_input;
  uint16_t mini_io_output;
};

/**
 * IOCTL_GET_VERSION
 */
struct DriverVersion
{
  int driver_major;
  int driver_minor;
  int driver_revision;
  int driver_build;
  std::string fpga_main;
  std::string fpga_main_backup;
  std::string mcu;
  std::string safety_mcu;
};

/**
 * IOCTL_SRV_UPDATE
 */
struct DriverCommandInfo
{
  uint32_t result = 0;
  int queue_num = 0;
  bool stay_here = false;
};

class Driver
{
public:
  static const char* TAG;

  Driver(const std::shared_ptr<Cobotta>& parent);
  virtual ~Driver() = default;

  int openDeviceFile() throw(CobottaException, std::runtime_error);
  void closeDeviceFile(int fd);

  bool update(const std::array<uint32_t, ARM_MAX>& driver_queue_size, bool driver_error, bool driver_fatal_error,
              bool ip_reset_button);
  bool isTargetVersion() const;
  void printVersion() const;
  void clearError() throw(CobottaException, std::runtime_error);
  StateCode dequeue(long arm_no) throw(CobottaException, std::runtime_error, std::invalid_argument);
  DriverStateInfo receiveAllState() throw(CobottaException, std::runtime_error);
  uint32_t getStateQueueSize(long arm_no) const throw(std::invalid_argument);
  DriverVersion getVersion() const;
  bool isError() const;
  bool isFatalError() const;
  bool isIpResetButtonOn() const;

  static void checkArmNo(long arm_no) throw(std::invalid_argument);
  static void checkJointNo(long joint_no) throw(std::invalid_argument);

  static struct DriverCommandInfo writeHwUpdate(int fd, const SRV_COMM_SEND& send_data) throw(std::runtime_error);
  static SRV_COMM_RECV readHwEncoder(int fd, long arm_no) throw(CobottaException, std::runtime_error,
                                                                std::invalid_argument);
  static void writeHwAcyclicCommAll(int fd, uint16_t address, std::array<uint16_t, JOINT_MAX + 1> send_values,
                                    std::array<uint16_t, JOINT_MAX + 1>& recv_values) throw(CobottaException,
                                                                                            std::runtime_error);
  static void writeHwAcyclicComm(int fd, long arm_no, long joint_no, uint16_t address, uint16_t send_value,
                                 uint16_t& recv_value) throw(CobottaException, std::invalid_argument,
                                                             std::runtime_error);

private:
  static DriverVersion readHwVersion(int fd) throw(CobottaException, std::runtime_error);
  static void writeHwClear(int fd) throw(CobottaException, std::runtime_error);
  static StateCode readHwQueue(int fd, long arm_no) throw(CobottaException, std::runtime_error);
  static DriverStateInfo readHwState(int fd) throw(CobottaException, std::runtime_error);

  std::shared_ptr<Cobotta> parent_;

  std::array<uint32_t, ARM_MAX> queue_size_;

  bool ip_reset_button_;
  bool error_;
  bool fatal_error_;
  DriverVersion version_;
};

} /* namespace cobotta */
} /* namespace denso_cobotta_lib */

#endif /* DRIVER_H_ */
