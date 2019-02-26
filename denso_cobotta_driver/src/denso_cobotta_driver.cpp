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

#include "denso_cobotta_driver/denso_cobotta_driver.h"
#include "denso_cobotta_driver/cobotta_common.h"
#include "denso_cobotta_driver/cobotta_state.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "denso_cobotta_driver");
  ros::NodeHandle nh;

  // Initialize driver
  denso_cobotta_driver::DensoCobottaDriver driver;
  bool ret = driver.initialize(nh);
  if (!ret)
  {
    ROS_ERROR_STREAM("Failed to initialize COBOTTA device driver.");
    return 1;
  }
  ROS_INFO_STREAM("Finish to initialize COBOTTA device driver.");

  ros::Rate loop_rate(100);  // default: rate 125
  while (ros::ok())
  {
    driver.checkState();
    driver.fetchMiniInput();
    driver.publishState();

    ros::spinOnce();
    loop_rate.sleep();
  }

  driver.terminate();

  return 0;
}

namespace denso_cobotta_driver
{
using namespace cobotta_common;
using namespace cobotta_state;

const uint16_t DensoCobottaDriver::ACYCLIC_COMM_ADDRESS[2] = { 33283, 33284 };
const uint16_t DensoCobottaDriver::ACYCLIC_COMM_VALUE[2][9] = { { 16, 53, 47, 43, 40, 61, 37, 37, 10240 },
                                                                { 802, 1066, 1034, 598, 463, 489, 3986, 3986, 1250 } };

DensoCobottaDriver::DensoCobottaDriver()
{
  position_button_state_.data = false;
  open_button_state_.data = false;
  close_button_state_.data = false;
  level5_error_flag = false;
  safe_state_error_flag = false;
  robot_error_flag = false;
}

DensoCobottaDriver::~DensoCobottaDriver()
{
}

bool DensoCobottaDriver::initialize(ros::NodeHandle& nh)
{
  errno = 0;
  fd_ = open(PATH_DEVFILE.c_str(), O_RDWR);
  if (fd_ < 0)
  {
    ROS_ERROR("open(%s): %s", PATH_DEVFILE.c_str(), std::strerror(errno));
    return false;
  }

#if 1
  // LED::White
  this->setLED(this->LedColor::white);
#endif

  // driver version
  struct LinuxDriverVersion linux_driver_version;
  if (this->getDriverVersion(&linux_driver_version))
  {
      ROS_INFO("DensoCobottaDriver: Linux driver version is %d.%d.%d-%d",
	       linux_driver_version.major,
	       linux_driver_version.minor,
	       linux_driver_version.rev,
	       linux_driver_version.build);
  }

  // Service server
  sv_set_motor_ = nh.advertiseService("set_motor_state", &DensoCobottaDriver::setMotorStateCB, this);
  sv_get_motor_ = nh.advertiseService("get_motor_state", &DensoCobottaDriver::getMotorStateCB, this);
  sv_set_brake_ = nh.advertiseService("set_brake_state", &DensoCobottaDriver::setBrakeStateCB, this);
  sv_get_brake_ = nh.advertiseService("get_brake_state", &DensoCobottaDriver::getBrakeStateCB, this);
  sv_clear_error_ = nh.advertiseService("clear_error", &DensoCobottaDriver::clearErrorCB, this);
  sv_clear_robot_error_ = nh.advertiseService("clear_robot_error", &DensoCobottaDriver::clearRobotErrorCB, this);
  sv_clear_safe_state_ = nh.advertiseService("clear_safe_state", &DensoCobottaDriver::clearSafeStateCB, this);
  sv_set_LED_ = nh.advertiseService("set_LED_state", &DensoCobottaDriver::setLEDStateCB, this);

  // Publisher
  pub_position_button_ = nh.advertise<std_msgs::Bool>("position_button", 1);
  pub_open_button_ = nh.advertise<std_msgs::Bool>("open_button", 1);
  pub_close_button_ = nh.advertise<std_msgs::Bool>("close_button", 1);
  pub_miniIO_input_ = nh.advertise<std_msgs::UInt16>("miniIO_input", 1);
  pub_robot_state_ = nh.advertise<denso_cobotta_driver::RobotState>("robot_state", 64);
  pub_safe_state_ = nh.advertise<denso_cobotta_driver::SafeState>("safe_state", 64);

  // Subscriber
  sub_miniIO_output_ = nh.subscribe("miniIO_output", 10, &DensoCobottaDriver::subMiniIOOutputCB, this);

  IOCTL_DATA_CB_ACYCLIC_COMM acyclic_comm;
  acyclic_comm.InLength = sizeof(SRV_CB_ACYCLIC_2_ALL_REQ);

  // Set acyclic comm parameters.
  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 9; j++)
    {
      acyclic_comm.req_2_all.Data[i][j].Address = ACYCLIC_COMM_ADDRESS[i];
      acyclic_comm.req_2_all.Data[i][j].Value = ACYCLIC_COMM_VALUE[i][j];
    }
  }

  errno = 0;
  int ret = ioctl(fd_, COBOTTA_IOCTL_CB_ACYCLIC_COMM, &acyclic_comm);
  if (ret != 0)
  {
    ROS_ERROR("ioctl(CB_ACYCLIC_COMM): %s (ret=%d errno=%d)", std::strerror(errno), ret, errno);
    return false;
  }

#if 1 // control node call clear_error
  // possible to motor_on.
  if (!this->clearError())
    return false;

  // XXX: If driver node is dead on motor_on failed, users can't see checkstate.
  if (!this->motorON())
	  return true;

  this->setLED(this->LedColor::green);
#endif

  return true;
}

void DensoCobottaDriver::terminate()
{
  this->motorOFF();
  close(fd_);
}

bool DensoCobottaDriver::getDriverVersion(struct LinuxDriverVersion* version)
{
  int ret;
  IOCTL_DATA_GET_VERSION ver;

  errno = 0;
  ret = ioctl(this->fd_, COBOTTA_IOCTL_GET_VERSION, &ver);
  if (ret != 0)
  {
    ROS_ERROR("ioctl(GET_VERSION): %s (ret=%d errno=%d)", std::strerror(errno), ret, errno);
    return false;
  }
  version->major = (ver.driver_ver >> 24) & 0xff;
  version->minor = (ver.driver_ver >> 16) & 0xff;
  version->rev = (ver.driver_ver >> 8) & 0xff;
  version->build = ver.driver_ver & 0xff;

  return true;
}
bool DensoCobottaDriver::motorON()
{
  int ret;
  long result;

  result = 0;
  errno = 0;
  ret = ioctl(fd_, COBOTTA_IOCTL_MOTOR_ON, &result);
  if (ret != 0)
  {
    ROS_ERROR("ioctl(MOTOR_ON): %s (ret=%d errno=%d)", std::strerror(errno), ret, errno);
    return false;
  }
  if (result != 0)
  {
    ROS_ERROR("DensoCobottaDriver: Failed to motorON. %s (result=0x%lX)",
              CobottaState::getMessage(static_cast<uint32_t>(result)).c_str(), result);
    return false;
  }

  return true;
}

bool DensoCobottaDriver::motorOFF()
{
  int ret;
  errno = 0;
  ret = ioctl(fd_, COBOTTA_IOCTL_MOTOR_OFF, 0);
  if (ret != 0)
  {
    ROS_ERROR("ioctl(MOTOR_OFF): %s (ret=%d errno=%d)", std::strerror(errno), ret, errno);
    return false;
  }

  return true;
}

bool DensoCobottaDriver::motorState(long* state)
{
  int ret;

  errno = 0;
  ret = ioctl(fd_, COBOTTA_IOCTL_MOTOR_STATE, state);
  if (ret != 0)
  {
    ROS_ERROR("ioctl(MOTOR_STATE): %s (ret=%d errno=%d state=%ld)", std::strerror(errno), ret, errno, *state);
    return false;
  }

  return true;
}

bool DensoCobottaDriver::setBrake()
{
  int ret;

  brake_state_.ArmNo = 0;
  errno = 0;
  ret = ioctl(fd_, COBOTTA_IOCTL_SRV_SETBRAKE, &brake_state_);
  if (ret != 0)
  {
    ROS_ERROR("ioctl(SRV_SETBRAKE): %s (ret=%d errno=%d ArmNo=0)", std::strerror(errno), ret, errno);
    return false;
  }
  ROS_INFO("Success to set brake.");

  return true;
}

bool DensoCobottaDriver::getBrake()
{
  int ret;

  brake_state_.ArmNo = 0;
  errno = 0;
  ret = ioctl(fd_, COBOTTA_IOCTL_SRV_GETBRAKE, &brake_state_);
  if (ret != 0)
  {
    ROS_ERROR("ioctl(SRV_SETBRAKE): %s (ret=%d errno=%d ArmNo=0)", std::strerror(errno), ret, errno);
    return false;
  }

  return true;
}

bool DensoCobottaDriver::fetchMiniInput()
{
  unsigned short input;
  int ret;
  static std_msgs::UInt16 miniIO_input;

  errno = 0;
  ret = ioctl(fd_, COBOTTA_IOCTL_MINI_INPUT, &input);
  if (ret != 0)
  {
    ROS_ERROR("ioctl(MINI_INPUT): %s (ret=%d errno=%d)", std::strerror(errno), ret, errno);
    return false;
  }
  miniIO_input.data = input;
  pub_miniIO_input_.publish(miniIO_input);
  return true;
}

bool DensoCobottaDriver::checkState()
{
  int ret;
  static char reprace_str[256];
  static std::string message;
  static denso_cobotta_driver::RobotState robot_state_msg;
  static CobottaStateParam* state_param;
  static IOCTL_DATA_CHECKSTATE srv_state;

  /*
   * arm (arm=0)
   */
  while (1)
  {
    srv_state.Arm = 0;
    errno = 0;
    ret = ioctl(fd_, COBOTTA_IOCTL_SRV_CHECKSTATE, &srv_state);
    if (ret != 0)
    {
      ROS_ERROR("ioctl(SRV_CHECKSTATE): %s (ret=%d errno=%d ArmNo=0)", std::strerror(errno), ret, errno);
      return false;
    }

    if (srv_state.Err.ErrorCode == 0)
    {
      break;
    }

    // publish
    robot_state_msg.arm_no = 0;
    robot_state_msg.state_code = srv_state.Err.ErrorCode;
    robot_state_msg.state_subcode = srv_state.Err.SubCode;
    pub_robot_state_.publish(robot_state_msg);

    state_param = CobottaState::getState(srv_state.Err.ErrorCode, srv_state.Err.SubCode);
    if (state_param->getLevel() >= 5)
      this->level5_error_flag = true;
    if (state_param->getLevel() >= 1)
      this->robot_error_flag = true;
    if (!this->safe_state_error_flag && (state_param->getLedColor() != this->LedColor::no_change))
      this->setLED(state_param->getLedColor());
    state_param->putRosLog("DensoCobottaDriver[state/arm]");
    delete state_param;
    switch (srv_state.Err.ErrorCode)
    {
      case 0x0f200201:  // motor on
        break;
      case 0x0f200202:  // motor off
        break;
      case 0x0f200031:
        open_button_state_.data = true;
        break;
      case 0x0f200032:
        open_button_state_.data = false;
        break;
      case 0x0f200033:
        close_button_state_.data = true;
        break;
      case 0x0f200034:
        close_button_state_.data = false;
        break;
      case 0x0f200035:
        position_button_state_.data = true;
        break;
      case 0x0f200036:
        position_button_state_.data = false;
        break;
      default:
        break;
    }
  }

  /*
   * gripper (arm=1)
   */
  while (1)
  {
    srv_state.Arm = 1;
    errno = 0;
    ret = ioctl(fd_, COBOTTA_IOCTL_SRV_CHECKSTATE, &srv_state);
    if (ret != 0)
    {
      ROS_ERROR("ioctl(SRV_CHECKSTATE): %s (ret=%d errno=%d ArmNo=1)", std::strerror(errno), ret, errno);
      return false;
    }

    if (srv_state.Err.ErrorCode == 0)
    {
      break;
    }

    // publish
    robot_state_msg.arm_no = 1;
    robot_state_msg.state_code = srv_state.Err.ErrorCode;
    robot_state_msg.state_subcode = srv_state.Err.SubCode;
    pub_robot_state_.publish(robot_state_msg);

    state_param = CobottaState::getState(srv_state.Err.ErrorCode, srv_state.Err.SubCode);
    if (state_param->getLevel() >= 5)
      this->level5_error_flag = true;
    if (state_param->getLevel() >= 1)
      this->robot_error_flag = true;
    if (!this->safe_state_error_flag && (state_param->getLedColor() != this->LedColor::no_change))
      this->setLED(state_param->getLedColor());
    state_param->putRosLog("DensoCobottaDriver[state/gripper");
    delete state_param;
  }

  /*
   * safety-MCU
   */
  while (1)
  {
    static IOCTL_DATA_SAFETY_CHECKSTATE safe_state;

    memset(&safe_state, 0, sizeof(safe_state));
    errno = 0;
    ret = ioctl(fd_, COBOTTA_IOCTL_SAFETY_CHECKSTATE, &safe_state);
    if (ret)
    {
      ROS_ERROR("ioctl(SRV_CHECKSTATE): %s (ret=%d errno=%d)", std::strerror(errno), ret, errno);
      return false;
    }
    if (!safe_state.Err.ErrorCode)
    {
      break;
    }

    static denso_cobotta_driver::SafeState safe_state_msg;
    safe_state_msg.state_code = safe_state.Err.ErrorCode;
    safe_state_msg.state_subcode = safe_state.Err.SubCode;
    pub_safe_state_.publish(safe_state_msg);

    state_param = CobottaState::getState(safe_state.Err.ErrorCode, safe_state.Err.SubCode);
    if (state_param->getLevel() >= 1)
      this->safe_state_error_flag = true;
    if (state_param->getLevel() >= 5)
      this->level5_error_flag = true;
    if (state_param->getLedColor() != this->LedColor::no_change)
      this->setLED(state_param->getLedColor());
    state_param->putRosLog("DensoCobottaDriver[state/safety]");
    delete state_param;
  }

  return true;
}

bool DensoCobottaDriver::clearError()
{
  if (!this->clearRobotError())
    return false;
  if (!this->clearSafeState())
    return false;

  return true;
}

bool DensoCobottaDriver::clearRobotError()
{
  int ret;

  if (this->level5_error_flag)
  {
    ROS_FATAL("DensoCobottaDriver: Fatal error can't be cleared.");
    return false;
  }

  errno = 0;
  ret = ioctl(fd_, COBOTTA_IOCTL_CLR_ERROR);
  if (ret != 0)
  {
    ROS_ERROR("ioctl(CLR_ERROR): %s (ret=%d errno=%d)", std::strerror(errno), ret, errno);
    return false;
  }
  this->robot_error_flag = false;
  ROS_INFO("Success to clear_robot_error.");

  return true;
}

bool DensoCobottaDriver::clearSafeState()
{
  int ret;
  IOCTL_DATA_SAFETY_SEND snd;

  if (this->level5_error_flag)
  {
    ROS_FATAL("DensoCobottaDriver: Fatal error can't be cleared.");
    return false;
  }

  // Safety function clear error.
  //   SafetState => Standby.
  snd.Code = 0;
  errno = 0;
  ret = ioctl(fd_, COBOTTA_IOCTL_SAFETY_SEND, &snd);
  if (ret != 0)
  {
    ROS_ERROR("ioctl(SAFETY_SEND): %s (ret=%d errno=%d code=0)", std::strerror(errno), ret, errno);
    return false;
  }
  else if (snd.Result != 0)
  {
    ROS_ERROR("DensoCobottaDriver: clearSafeState() failed. %s (Code=Standby Result=0x%08lX)",
              CobottaState::getMessage(static_cast<uint32_t>(snd.Result)).c_str(), snd.Result);
    return false;
  }

  // Wait 1 sec for sure to transit safe state to stanby.
  ros::Duration(3.0).sleep();

  // Standby => Normal.
  snd.Code = 2;
  errno = 0;
  ret = ioctl(fd_, COBOTTA_IOCTL_SAFETY_SEND, &snd);
  if (ret != 0)
  {
    ROS_ERROR("ioctl(SAFETY_SEND): %s (ret=%d errno=%d Code=2", std::strerror(errno), ret, errno);
    return false;
  }
  else if (snd.Result != 0)
  {
    ROS_ERROR("DensoCobottaDriver: clearSafeState() failed. %s (Code=Normal Result=0x%08lX)",
              CobottaState::getMessage(static_cast<uint32_t>(snd.Result)).c_str(), snd.Result);
    return false;
  }

  // Wait 1 sec for sure to transit standby to normal.
  ros::Duration(1.0).sleep();
  this->safe_state_error_flag = false;
  ROS_INFO("Success to clear_safe_state: standby to normal.");

  return true;
}

bool DensoCobottaDriver::setLED(const uint8_t red, const uint8_t green, const uint8_t blue,
                                const uint8_t blink_rate)
{
  int ret;
  long value;
  IOCTL_DATA_PUTSTATE putState;

  value = (blink_rate << 24) | (blue << 0) | (green) << 8 | (red << 16);
  putState.Index = SRVSTATE_CB_LED;
  putState.Value = value;
  errno = 0;
  ret = ioctl(fd_, COBOTTA_IOCTL_SRV_PUTSTATE, &putState);
  if (ret != 0)
  {
    ROS_ERROR("ioctl(SRV_PUTSTATE): %s (ret=%d errno=%d Index=0x%08X Value=0x%08lX)", std::strerror(errno), ret, errno,
              putState.Index, putState.Value);
    return false;
  }

  return true;
}

bool DensoCobottaDriver::setLED(const uint64_t value)
{
  int ret;
  IOCTL_DATA_PUTSTATE putState;

  putState.Index = SRVSTATE_CB_LED;
  putState.Value = value;
  errno = 0;
  ret = ioctl(fd_, COBOTTA_IOCTL_SRV_PUTSTATE, &putState);
  if (ret != 0)
  {
    ROS_ERROR("ioctl(SRV_PUTSTATE): %s (ret=%d errno=%d Index=0x%08X Value=0x%08lX)", std::strerror(errno), ret, errno,
              putState.Index, putState.Value);
    return false;
  }
  return true;
}

void DensoCobottaDriver::publishState()
{
  pub_position_button_.publish(position_button_state_);
  pub_open_button_.publish(open_button_state_);
  pub_close_button_.publish(close_button_state_);
}

bool DensoCobottaDriver::setMotorStateCB(SetMotorState::Request& req, SetMotorState::Response& res)
{
  bool success;

  res.success = true;
  if (req.state)
  {
    success = motorON();
    if (!success)
    {
      res.success = false;
    }
  }
  else
  {
    success = motorOFF();
    if (!success)
    {
      res.success = false;
    }
  }

  return true;
}

bool DensoCobottaDriver::getMotorStateCB(GetMotorState::Request& /* req */, GetMotorState::Response& res)
{
  bool success;
  long state;

  success = motorState(&state);
  if (success)
  {
    res.state = (int)state == 0 ? false : true;
    res.success = true;
  }
  else
  {
    res.success = false;
  }
  return true;
}

bool DensoCobottaDriver::setBrakeStateCB(SetBrakeState::Request& req, SetBrakeState::Response& res)
{
  bool success;

  brake_state_.ArmNo = 0;
  res.success = true;

  if (req.state.size() != CONTROL_JOINT_MAX)
  {
    ROS_ERROR("DensoCobottaDriver: Failed to set_brake_state. "
              "The nubmer of joint is invalid. (size=%ld)",
              req.state.size());
    res.success = false;
    return true;
  }

  for (int i = 0; i < CONTROL_JOINT_MAX; i++)
  {
    if (req.state[i])
    {
      brake_state_.BrakeState[i] = SRV_BRAKE_LOCK;
    }
    else
    {
      brake_state_.BrakeState[i] = SRV_BRAKE_RELEASE;
    }
  }

  success = setBrake();
  if (!success)
  {
    res.success = false;
  }

  return true;
}

bool DensoCobottaDriver::getBrakeStateCB(GetBrakeState::Request& /* req */, GetBrakeState::Response& res)
{
  bool success;

  res.success = true;
  success = getBrake();
  if (!success)
  {
    res.success = false;
  }
  else
  {
    res.state.resize(CONTROL_JOINT_MAX);
    for (int i = 0; i < CONTROL_JOINT_MAX; i++)
    {
      res.state[i] = (int)brake_state_.BrakeState[i] == SRV_BRAKE_LOCK ? true : false;
    }
  }

  return true;
}

bool DensoCobottaDriver::clearErrorCB(ClearError::Request& /* req */, ClearError::Response& res)
{
  bool success;

  // IOCTL_CLEAR_ERROR + IOCTL_SAFETY_ERROR
  success = this->clearError();
  if (!success)
  {
    res.success = false;
    return true;
  }

  if (!this->level5_error_flag)
    this->setLED(this->LedColor::green);

  res.success = true;
  return true;
}

bool DensoCobottaDriver::clearRobotErrorCB(ClearError::Request& /* req */, ClearError::Response& res)
{
  bool success;

  // IOCTL_CLEAR_ERROR
  success = this->clearRobotError();
  if (!success)
  {
    res.success = false;
    return true;
  }

  if (!this->level5_error_flag && !this->safe_state_error_flag)
    this->setLED(this->LedColor::green);

  res.success = true;
  return true;
}

bool DensoCobottaDriver::clearSafeStateCB(ClearError::Request& /* req */, ClearError::Response& res)
{
  bool success;

  // IOCTL_CLEAR_SAFETY_ERROR
  success = this->clearSafeState();
  if (!success)
  {
    res.success = false;
    return true;
  }
  res.success = true;

  if (this->level5_error_flag)
    return true;
  if (this->robot_error_flag)
  {
    this->setLED(this->LedColor::yellow);
    return true;
  }
  this->setLED(this->LedColor::green);

  return true;
}

bool DensoCobottaDriver::setLEDStateCB(SetLEDState::Request& req, SetLEDState::Response& res)
{
  bool success;

  res.success = true;
  success = setLED(req.red, req.green, req.blue, req.blink_rate);
  if (!success)
  {
    res.success = false;
  }

  return true;
}

void DensoCobottaDriver::subMiniIOOutputCB(const std_msgs::UInt16::ConstPtr& msg)
{
  int ret;
  IOCTL_DATA_MINI_OUTPUT output;

  output.Data = msg->data;
  output.Mask = 0xFFFF;
  ROS_INFO_STREAM("DensoCobottaDriver: output.Data is " << output.Data);
  errno = 0;
  ret = ioctl(fd_, COBOTTA_IOCTL_MINI_OUTPUT, &output);
  if (ret != 0)
  {
    ROS_ERROR("ioctl(MINI_OUTPUT): %s (ret=%d errno=%d Data=0x%X Mask=0x%X)", std::strerror(errno), ret, errno,
              output.Data, output.Mask);
  }
}
}  // namespace denso_cobotta_driver
