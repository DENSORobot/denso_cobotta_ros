/*
 * driver.h: 分類できないものすべて
 *
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
  void sendParametersAll(uint16_t address, std::array<uint16_t, JOINT_MAX + 1> values) throw(CobottaException,
                                                                                             std::runtime_error);
  void sendParameters(long arm_no, long joint_no, uint16_t address,
                      uint16_t value) throw(CobottaException, std::runtime_error, std::invalid_argument);

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
  static long readHwGripperState(int fd) throw(CobottaException, std::runtime_error);

private:
  static DriverVersion readHwVersion(int fd) throw(CobottaException, std::runtime_error);
  static void writeHwClear(int fd) throw(CobottaException, std::runtime_error);
  static StateCode readHwQueue(int fd, long arm_no) throw(CobottaException, std::runtime_error);
  static DriverStateInfo readHwState(int fd) throw(CobottaException, std::runtime_error);
  static void writeHwAcyclicCommAll(int fd, uint16_t address,
                                    std::array<uint16_t, JOINT_MAX + 1> values) throw(CobottaException,
                                                                                      std::runtime_error);
  static void writeHwAcyclicComm(int fd, long arm_no, long joint_no, uint16_t address,
                                 uint16_t value) throw(CobottaException, std::runtime_error);

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
