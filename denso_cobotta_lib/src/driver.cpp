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
#include <fcntl.h>
#include <sys/ioctl.h>
#include "denso_cobotta_lib/driver.h"
#include "denso_cobotta_lib/cobotta_ioctl.h"
#include "denso_cobotta_lib/cobotta_common.h"

namespace denso_cobotta_lib
{
namespace cobotta
{
const char* Driver::TAG = "Driver";

/**
 * @brief Constructs a Driver object.
 *
 * @param Cobotta object
 */
Driver::Driver(const std::shared_ptr<Cobotta>& parent)
{
  parent_ = parent;
}

/**
 * @brief Open device file.
 *
 * @return int File descriptor.
 * @exception CobottaException An error defined by COBOTTA
 * @exception RuntimeError An other error
 */
int Driver::openDeviceFile() throw(CobottaException, std::runtime_error)
{
  int fd;
  int myerrno;

  errno = 0;
  fd = open(cobotta_common::PATH_DEVFILE.c_str(), O_RDWR);
  myerrno = errno;
  if (fd < 0)
  {
    throw std::runtime_error(std::strerror(myerrno));
  }
  // Read version information.
  version_ = Driver::readHwVersion(fd);

  return fd;
}

/**
 * @brief Close device file.
 *
 * @param fd File descriptor.
 */
void Driver::closeDeviceFile(int fd)
{
  close(fd);
}

/**
 * @brief Update Driver class state.
 *
 * @param driver_queue_size the size of driver queue.
 * @param driver_error If an error has occurred, it should be set true.
 * @param driver_fatal_error If a fatal error has occurred, it should be set true.
 * @param ip_reset_button If ip reset button is on, it should be set true.
 * @return If it is true, Driver class's state is changed.
 */
bool Driver::update(const std::array<uint32_t, ARM_MAX>& driver_queue_size, bool driver_error, bool driver_fatal_error,
                    bool ip_reset_button)
{
  bool changed = false;

  changed |= queue_size_ != driver_queue_size;
  queue_size_ = driver_queue_size;

  changed |= ip_reset_button_ != ip_reset_button;
  ip_reset_button_ = ip_reset_button;

  changed |= error_ != driver_error;
  error_ = driver_error;

  changed |= fatal_error_ != driver_fatal_error;
  fatal_error_ = driver_fatal_error;

  changed |= ip_reset_button_ != ip_reset_button;
  ip_reset_button_ = ip_reset_button;

  return changed;
}

/**
 * @brief Check version
 *
 * Driver's major version and minor version must match
 *
 * @return If it is true, driver version is match.
 */
bool Driver::isTargetVersion() const
{
  if (this->getVersion().driver_major != cobotta_common::DRIVER_VERSION_MAJOR)
    return false;
  if (this->getVersion().driver_minor != cobotta_common::DRIVER_VERSION_MINOR)
    return false;

  return true;
}

/**
 * @brief Output the all information of versions.
 *
 */
void Driver::printVersion() const
{
  ROS_INFO("%s: Linux driver: %d.%d.%d-%d (Requirements: %d.%d.N-M)", TAG, this->getVersion().driver_major,
           this->getVersion().driver_minor, this->getVersion().driver_revision, this->getVersion().driver_build,
           cobotta_common::DRIVER_VERSION_MAJOR, cobotta_common::DRIVER_VERSION_MINOR);
  ROS_INFO("%s: FPGA MAIN: %s", TAG, this->getVersion().fpga_main.c_str());
  ROS_INFO("%s: FPGA MAIN backup: %s", TAG, this->getVersion().fpga_main_backup.c_str());
  ROS_INFO("%s: ServoMCU: %s", TAG, this->getVersion().mcu.c_str());
  ROS_INFO("%s: SafetyMCU: %s", TAG, this->getVersion().safety_mcu.c_str());
}

/**
 * @brief Clear non-safety errors.
 *
 * Execute COBOTTA_IOCTL_CLR_ERROR.
 * Before execute this method, it must be executed Driver::dequeue until dequeue size is 0.
 * This method cannot clear fatal error.
 *
 * @exception CobottaException An error defined by COBOTTA
 * @exception RuntimeError An other error
 */
void Driver::clearError() throw(CobottaException, std::runtime_error)
{
  if (isFatalError() || parent_->getSafetyMcu()->isFatalError())
  {
    /* Fatal Error */
    throw CobottaException(0x85400001);
  }
  if (!this->isError())
  {
    ROS_INFO("%s: No robot error.", TAG);
    return;
  }

  /* waiting for zero of state queue */
  for (long i = 0; i < ARM_MAX; i++)
  {
    while (this->getStateQueueSize(i))
    {
      ros::Duration(cobotta_common::getPeriod()).sleep();
    }
  }

  writeHwClear(parent_->getFd());
  /* sync */
  while (this->isError())
  {
    if (this->isFatalError())
      throw CobottaException(0x854000F0); /* Fatal error occurred. */

    ros::Duration(cobotta_common::getPeriod()).sleep();
  }
  ROS_INFO("%s: ...Clearing error has done.", TAG);
}

/**
 * @brief Dequeue a state queue of driver.
 *
 * Execute COBOTTA_IOCTL_SRV_CHECKSTATE.
 *
 * @param arm_no
 * @return StateCode
 * @exception CobottaException An error defined by COBOTTA
 * @exception RuntimeError An other error
 * @exception InvalidArgument arm_no is invalid.
 */
StateCode Driver::dequeue(long arm_no) throw(CobottaException, std::runtime_error, std::invalid_argument)
{
  Driver::checkArmNo(arm_no);
  return Driver::readHwQueue(parent_->getFd(), arm_no);
}

/**
 * @brief Check and get DriverStateInfo.
 *
 * Execute COBOTTA_IOCTL_GET_ALLSTATE.
 *
 * @return DriverStateInfo
 * @exception CobottaException An error defined by COBOTTA
 * @exception RuntimeError An other error
 */
DriverStateInfo Driver::receiveAllState() throw(CobottaException, std::runtime_error)
{
  return Driver::readHwState(parent_->getFd());
}

/**
 * @brief Read the information of driver version.
 *
 * Execute IOCTL_GET_VERSION.
 *
 * @param fd
 * @return DriverVersion
 * @exception CobottaException An error defined by COBOTTA
 * @exception RuntimeError An other error
 */
DriverVersion Driver::readHwVersion(int fd) throw(CobottaException, std::runtime_error)
{
  int ret;
  int myerrno;
  IOCTL_DATA_GET_VERSION ver{ 0 };

  errno = 0;
  ret = ioctl(fd, COBOTTA_IOCTL_GET_VERSION, &ver);
  myerrno = errno;
  if (ret != 0)
  {
    if (myerrno == ECANCELED)
    {
      throw CobottaException(ver.result);
    }
    throw std::runtime_error(std::strerror(myerrno));
  }
  DriverVersion driver_ver{ 0 };

  driver_ver.driver_major = (ver.driver_ver >> 24) & 0xff;
  driver_ver.driver_minor = (ver.driver_ver >> 16) & 0xff;
  driver_ver.driver_revision = (ver.driver_ver >> 8) & 0xff;
  driver_ver.driver_build = ver.driver_ver & 0xff;
  driver_ver.fpga_main = ver.fpga_ver[0];
  driver_ver.fpga_main_backup = ver.fpga_ver[1];

  std::stringstream ss1, ss2;
  ss1 << std::setfill('0') << std::right << std::hex << std::uppercase << std::setw(4) << ver.servo_mcu_ver[0];
  ss2 << std::setfill('0') << std::right << std::hex << std::uppercase << std::setw(4) << ver.safety_mcu_ver[0][0]
      << "," << std::setw(4) << ver.safety_mcu_ver[0][1];
  for (int i = 1; i < JOINT_MAX + 1; i++)
  {
    ss1 << "-" << std::setw(4) << ver.servo_mcu_ver[i];
    ss2 << "-" << std::setw(4) << ver.safety_mcu_ver[i][0] << "," << std::setw(4) << ver.safety_mcu_ver[i][1];
  }
  driver_ver.mcu = ss1.str();
  driver_ver.safety_mcu = ss2.str();
  return driver_ver;
}

/**
 * [ASYNC] COBOTTA_IOCTL_CLR_ERROR
 * @param fd file descriptor
 * @exception CobottaException An error defined by COBOTTA
 * @exception RuntimeError An other error
 */
void Driver::writeHwClear(int fd) throw(CobottaException, std::runtime_error)
{
  int ret;
  int myerrno;
  IOCTL_DATA_RESULT dat{ 0 };

  errno = 0;
  ret = ioctl(fd, COBOTTA_IOCTL_CLR_ERROR, &dat);
  myerrno = errno;
  ROS_DEBUG("%s: COBOTTA_IOCTL_CLR_ERROR ret=%d errno=%d result=%lX", TAG, ret, myerrno, dat.result);
  if (ret)
  {
    if (myerrno == ECANCELED)
    {
      throw CobottaException(dat.result);
    }
    throw std::runtime_error(std::strerror(myerrno));
  }
}

/**
 * [ASYNC] COBOTTA_IOCTL_SRV_CHECKSTATE
 * @param fd file descriptor
 * @return error code
 * @exception CobottaException An error defined by COBOTTA
 * @exception RuntimeError An other error
 */
StateCode Driver::readHwQueue(int fd, long arm_no) throw(CobottaException, std::runtime_error)
{
  int ret;
  int myerrno;
  IOCTL_DATA_CHECKSTATE dat{ 0 };

  dat.arm_no = arm_no;
  errno = 0;
  ret = ioctl(fd, COBOTTA_IOCTL_SRV_CHECKSTATE, &dat);
  myerrno = errno;
  if (ret)
  {
    if (myerrno == ECANCELED)
    {
      throw CobottaException(dat.result);
    }
    throw std::runtime_error(std::strerror(myerrno));
  }
  StateCode code{ 0 };
  code.main_code = dat.err.error_code;
  code.sub_code = dat.err.sub_code;
  return code;
}

/**
 * [ASYNC] COBOTTA_IOCTL_GET_ALLSTATE
 * @param fd file descriptor
 * @return DriverStateInfo
 * @exception CobottaException An error defined by COBOTTA
 * @exception RuntimeError An other error
 */
DriverStateInfo Driver::readHwState(int fd) throw(CobottaException, std::runtime_error)
{
  int ret;
  int myerrno;
  IOCTL_DATA_GET_ALLSTATE dat{ 0 };

  {
    errno = 0;
    ret = ioctl(fd, COBOTTA_IOCTL_GET_ALLSTATE, &dat);
    myerrno = errno;
  }
  if (ret)
  {
    if (myerrno == ECANCELED)
    {
      throw CobottaException(dat.result);
    }
    throw std::runtime_error(std::strerror(myerrno));
  }

  DriverStateInfo info;
  for (int i = 0; i < ARM_MAX; i++)
  {
    info.driver_queue_size[i] = dat.checkstate_num[i];
  }
  info.safety_mcu_queue_size = dat.safety_checkstate_num;
  info.motor_state = dat.motor_state;
  info.safety_mcu_state = dat.safety_state;
  for (int i = 0; i < ARM_MAX; i++)
  {
    for (int j = 0; j < JOINT_MAX; j++)
    {
      info.brake_state[i][j] = dat.brake_state[i][j];
    }
  }
  info.emergency_stop = dat.emgstop_state != 0;
  info.protection_stop = dat.protectstop_state != 0;
  info.function_button = dat.function_btn_state != 0;
  info.plus_button = dat.hand_open_btn_state != 0;
  info.minus_button = dat.hand_close_btn_state != 0;
  info.ip_reset_button = dat.ip_reset_btn_state != 0;
  info.gripper_state = dat.gripper_state != 0;
  info.driver_fatal_error = dat.fatal_error & 0b01;
  info.driver_error = dat.error != 0;
  info.safety_mcu_fatal_error = dat.fatal_error & 0b10;
  info.safety_mcu_error = dat.safety_error != 0;
  info.mini_io_input = dat.mini_in;
  info.mini_io_output = dat.mini_out;
  info.gripper_state = dat.gripper_state != 0;

  return info;
}

/**
 * [ASYNC] COBOTTA_IOCTL_CB_ACYCLIC_COMM
 * @param fd file descriptor
 * @exception CobottaException An error defined by COBOTTA
 * @exception RuntimeError An other error
 */
void Driver::writeHwAcyclicCommAll(int fd, uint16_t address, std::array<uint16_t, JOINT_MAX + 1> send_values,
                                   std::array<uint16_t, JOINT_MAX + 1>& recv_values) throw(CobottaException,
                                                                                           std::runtime_error)
{
  int ret;
  int myerrno;
  static const int length = JOINT_MAX + 1;
  IOCTL_DATA_CB_ACYCLIC_COMM dat{ 0 };

  dat.in_length = sizeof(SRV_CB_ACYCLIC_1_ALL_REQ);
  for (int j = 0; j < length; j++)
  {
    dat.req_1_all.data[j].address = address;
    dat.req_1_all.data[j].value = send_values[j];
  }

  errno = 0;
  ret = ioctl(fd, COBOTTA_IOCTL_CB_ACYCLIC_COMM, &dat);
  myerrno = errno;
  if (ret != 0)
  {
    if (myerrno == ECANCELED)
    {
      throw CobottaException(dat.result_1_1.result);
    }
    throw std::runtime_error(std::strerror(myerrno));
  }
  for (int j = 0; j < length; j++)
  {
    recv_values[j] = dat.result_1_all.data[j].value;
  }
}

/**
 * [ASYNC] writeHwAcyclicComm
 * @param fd file descriptor
 * @exception CobottaException An error defined by COBOTTA
 * @exception InvalidArgument An other error
 * @exception RuntimeError An other error
 */
void Driver::writeHwAcyclicComm(int fd, long arm_no, long joint_no, uint16_t address, uint16_t send_value,
                                uint16_t& recv_value) throw(CobottaException, std::invalid_argument, std::runtime_error)
{
  int ret;
  int myerrno;
  IOCTL_DATA_CB_ACYCLIC_COMM dat{ 0 };

  Driver::checkArmNo(arm_no);
  Driver::checkJointNo(joint_no);

  dat.in_length = sizeof(SRV_CB_ACYCLIC_1_1_REQ);
  dat.req_1_1.arm_no = arm_no;
  dat.req_1_1.joint_no = joint_no;
  dat.req_1_1.data.address = address;
  dat.req_1_1.data.value = send_value;

  errno = 0;
  ret = ioctl(fd, COBOTTA_IOCTL_CB_ACYCLIC_COMM, &dat);
  myerrno = errno;
  if (ret != 0)
  {
    if (myerrno == ECANCELED)
    {
      throw CobottaException(dat.result_1_1.result);
    }
    throw std::runtime_error(std::strerror(myerrno));
  }
  recv_value = dat.result_1_1.data.value;
}

/**
 * [ASYNC] COBOTTA_IOCTL_SRV_UPDATE
 * @param fd file descriptor
 * @exception RuntimeError An other error
 */
struct DriverCommandInfo Driver::writeHwUpdate(int fd, const SRV_COMM_SEND& send_data) throw(std::runtime_error)
{
  IOCTL_DATA_UPDATE dat{ 0 };
  DriverCommandInfo info;

  dat.send = send_data;

  errno = 0;
  int ret = ioctl(fd, COBOTTA_IOCTL_SRV_UPDATE, &dat);
  int myerrno = errno;
  if (ret)
  {
    if (myerrno == ECANCELED)
    {
      info.result = dat.recv.result;
      info.queue_num = (dat.recv.buff_state & 0xffff);
      info.stay_here = dat.recv.buff_state && 0x00010000;
    }
    else
      throw std::runtime_error(std::strerror(myerrno));
  }

  return info;
}

SRV_COMM_RECV Driver::readHwEncoder(int fd, long arm_no) throw(CobottaException, std::runtime_error,
                                                               std::invalid_argument)
{
  IOCTL_DATA_GETENC dat{ 0 };
  dat.arm_no = arm_no;
  Driver::checkArmNo(arm_no);

  errno = 0;
  int ret = ioctl(fd, COBOTTA_IOCTL_SRV_GETENC, &dat);
  int myerrno = errno;
  if (ret != 0)
  {
    if (myerrno == ECANCELED)
    {
      throw CobottaException(dat.recv.result);
    }
    else
    {
      throw std::runtime_error(std::strerror(myerrno));
    }
  }
  return dat.recv;
}

uint32_t Driver::getStateQueueSize(long arm_no) const throw(std::invalid_argument)
{
  Driver::checkArmNo(arm_no);
  return queue_size_[arm_no];
}

void Driver::checkArmNo(long arm_no) throw(std::invalid_argument)
{
  if (arm_no < 0 || arm_no > ARM_MAX)
  {
    throw std::invalid_argument("Invalid arm_no: " + std::to_string(arm_no) + " is out of range(0 <= arm_no < " +
                                std::to_string(ARM_MAX) + ").");
  }
}
void Driver::checkJointNo(long joint_no) throw(std::invalid_argument)
{
  if (joint_no < 0 || joint_no > JOINT_MAX)
  {
    throw std::invalid_argument("Invalid joint_no: " + std::to_string(joint_no) + " is out of range(0 <= joint_no < " +
                                std::to_string(JOINT_MAX) + ").");
  }
}

DriverVersion Driver::getVersion() const
{
  return version_;
}

bool Driver::isError() const
{
  return error_;
}

bool Driver::isFatalError() const
{
  return fatal_error_;
}

bool Driver::isIpResetButtonOn() const
{
  return ip_reset_button_;
}

} /* namespace cobotta */
} /* namespace denso_cobotta_lib */
