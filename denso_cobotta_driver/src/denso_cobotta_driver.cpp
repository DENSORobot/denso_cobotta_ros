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
#include <algorithm>
#include <iterator>
#include <thread>
#include <stdexcept>
#include <signal.h>

#include "denso_cobotta_driver/denso_cobotta_driver.h"
#include "denso_cobotta_lib/cobotta.h"

void signalHandler(int sig)
{
  int fd;
  fd = open(cobotta_common::PATH_DEVFILE.c_str(), O_RDWR);
  if (fd)
  {
    denso_cobotta_lib::cobotta::Motor::sendStop(fd);
    close(fd);
  }
  ROS_INFO("DensoCobotttaDriver has stopped.");
  ros::shutdown();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "denso_cobotta_driver");
  ros::NodeHandle nh;

  /*
   * Initialize
   */
  denso_cobotta_driver::DensoCobottaDriver driver;
  bool ret = driver.initialize(nh);
  if (!ret)
  {
    ROS_ERROR_STREAM("Failed to initialize COBOTTA device driver.");
    return 1;
  }
  ROS_INFO_STREAM("Finish to initialize COBOTTA device driver.");

  /* for Ctrl+C */
  signal(SIGINT, signalHandler);

  /*
   * Updating state on another thread
   */
  std::thread update_thread([&driver]() {
    ros::Rate update_rate(100);
    while (ros::ok()) {
      driver.update();
      update_rate.sleep();
    }
  });

  /*
   * main thread
   */
  driver.start();
  ros::spin();
  driver.stop();
  ros::shutdown();
  update_thread.join();
  driver.terminate();

  return 0;
}

namespace denso_cobotta_driver
{
using namespace denso_cobotta_lib::cobotta;
DensoCobottaDriver::DensoCobottaDriver()
{
  cobotta_ = std::make_shared<Cobotta>();
  force_clear_flag_ = false;
}

/**
 * Initialize this node
 * @param nh Node Handle
 * @return true success to initialize
 * @return false failed to initialize
 */
bool DensoCobottaDriver::initialize(ros::NodeHandle& nh)
{
  try
  {
    if (!cobotta_->initialize())
      return false;

    cobotta_->getLed()->forceChange(static_cast<uint32_t>(LedColorTable::WHITE));

    // Service server
    sv_set_motor_ = nh.advertiseService("set_motor_state", &DensoCobottaDriver::setMotorStateSv, this);
    sv_get_motor_ = nh.advertiseService("get_motor_state", &DensoCobottaDriver::getMotorStateSv, this);
    sv_set_brake_ = nh.advertiseService("set_brake_state", &DensoCobottaDriver::setBrakeStateSv, this);
    sv_get_brake_ = nh.advertiseService("get_brake_state", &DensoCobottaDriver::getBrakeStateSv, this);
    sv_clear_error_ = nh.advertiseService("clear_error", &DensoCobottaDriver::clearErrorSv, this);
    sv_clear_robot_error_ = nh.advertiseService("clear_robot_error", &DensoCobottaDriver::clearRobotErrorSv, this);
    sv_clear_safe_state_ = nh.advertiseService("clear_safe_state", &DensoCobottaDriver::clearSafeStateSv, this);
    sv_set_led_ = nh.advertiseService("set_LED_state", &DensoCobottaDriver::setLedStateSv, this);

    // Publisher
    pub_function_button_.init(nh, "function_button", 1);
    pub_plus_button_.init(nh, "plus_button", 1);
    pub_minus_button_.init(nh, "minus_button", 1);
    pub_mini_io_input_.init(nh, "miniIO_input", 1);
    pub_robot_state_.init(nh, "robot_state", 1);
    pub_safe_state_.init(nh, "safe_state", 1);

    // Subscriber
    sub_mini_io_output_ = nh.subscribe("miniIO_output", 10, &DensoCobottaDriver::miniIoOutputCallback, this);

    PublishInfo info = this->getCobotta()->update();
    this->publish(false, info);
  }
  catch (const CobottaException& e)
  {
    Message::putRosConsole(nullptr, e);
    return false;
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    return false;
  }

  return true;
}

/**
 * Start this node
 */
void DensoCobottaDriver::start()
{
  try
  {
    /* check */
    if (this->getCobotta()->getMotor()->isRunning())
      return;
    if (this->getCobotta()->getSafetyMcu()->isEmergencyButton())
    {
      // Turn OFF Emergency-stop and execute the command clear_safe_state.
      throw CobottaException(0x81400016);
    }
     if (this->getCobotta()->getSafetyMcu()->isProtectiveButton())
    {
       // Turn OFF Protective-stop signal to execute the command.
       throw CobottaException(0x81400019);
    }

    /* start... */
    if (this->isForceClearFlag())
    {
      this->getCobotta()->getSafetyMcu()->forceMoveToStandby();
    }
    if (!this->getCobotta()->getSafetyMcu()->isNormal())
      this->getCobotta()->getSafetyMcu()->moveToNormal();
    if (this->getCobotta()->getDriver()->isError())
      this->getCobotta()->getDriver()->clearError();

    /* motor on */
    this->getCobotta()->getMotor()->start();
  }
  catch (const CobottaException& e)
  {
    Message::putRosConsole(nullptr, e);
    return;
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    return;
  }
}

/**
 *
 */
void DensoCobottaDriver::stop()
{
  try
  {
    cobotta_->getMotor()->stop();
  }
  catch (const CobottaException& e)
  {
    Message::putRosConsole(nullptr, e);
    return;
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    return;
  }
}
/**
 * Terminate this
 */
void DensoCobottaDriver::terminate()
{
  try
  {
    cobotta_->terminate();
  }
  catch (const CobottaException& e)
  {
    Message::putRosConsole(nullptr, e);
    return;
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    return;
  }
}

/**
 * Update my state.
 */
void DensoCobottaDriver::update()
{
  try
  {
    /*
     * update
     */
    PublishInfo pi = this->getCobotta()->update();
    /*
     * Dequeue & Publish
     */
    this->publish(true, pi);
  }
  catch (const CobottaException& e)
  {
    Message::putRosConsole(nullptr, e);
    return;
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM_THROTTLE(1, e.what());
  }
}
/**
 * Publish my state.
 * @param sync true:sync false:async(first)
 */
void DensoCobottaDriver::publish(const bool sync, const PublishInfo info)
{
    bool lv4_error = false;
    bool lv5_error = false;
    bool watchdog_error = false;

    try
    {
      /* Driver */
      for (int i = 0; i < ARM_MAX; i++)
      {
        for (int j = 0; j < info.getDriverQueueSize(i); j++)
        {
          this->pub_robot_state_.lock();
          struct StateCode sc = this->getCobotta()->getDriver()->dequeue(i);
          std::string tag = "robot#" + std::to_string(j);
          this->putRosLog(tag.c_str(), sc.main_code, sc.sub_code);
          this->pub_robot_state_.msg_.state_code = sc.main_code;
          this->pub_robot_state_.msg_.state_subcode = sc.sub_code;
          this->pub_robot_state_.unlockAndPublish();

          auto msg = Message::getMessageInfo(sc.main_code);
          if (msg.level >= 4)
            lv4_error = true;
        }
      }
      /* safetyMCU */
      for (int i = 0; i < info.getSafetyMcuQueueSize(); i++)
      {
        this->pub_safe_state_.lock();
        struct StateCode sc = this->getCobotta()->getSafetyMcu()->dequeue();
        this->putRosLog("safety", sc.main_code, sc.sub_code);
        this->pub_safe_state_.msg_.state_code = sc.main_code;
        this->pub_safe_state_.msg_.state_subcode = sc.sub_code;
        this->pub_safe_state_.unlockAndPublish();

        auto msg = Message::getMessageInfo(sc.main_code);
        if (msg.level >= 4)
          lv4_error = true;
        if (msg.level >= 5)
          lv5_error = true;

        /* watchdog timer error */
        if (Message::isWatchdogTimerError(sc.main_code))
          watchdog_error = true;
      }
      /* motor off on Lv4 error */
      if (sync && lv4_error)
        this->getCobotta()->getMotor()->stop();
      /* On async, watchdog timer error */
      if (!sync && !lv5_error && watchdog_error)
        this->setForceClearFlag(true);

      /* other */
      this->pub_function_button_.lock();
      this->pub_function_button_.msg_.data = info.isFunctionButton();
      this->pub_function_button_.unlockAndPublish();
      this->pub_plus_button_.lock();
      this->pub_plus_button_.msg_.data = info.isPlusButton();
      this->pub_plus_button_.unlockAndPublish();
      this->pub_minus_button_.lock();
      this->pub_minus_button_.msg_.data = info.isMinusButton();
      this->pub_minus_button_.unlockAndPublish();
      this->pub_mini_io_input_.lock();
      this->pub_mini_io_input_.msg_.data = info.getMiniIo();
      this->pub_mini_io_input_.unlockAndPublish();
    }
    catch (const CobottaException& e)
    {
      Message::putRosConsole(nullptr, e);
      return;
    }
    catch (const std::exception& e)
    {
      ROS_ERROR_STREAM(e.what());
      return;
    }
}
/**
 * Output ROS_CONSOLE
 * @param tag
 * @param main_code
 * @param sub_code
 */
void DensoCobottaDriver::putRosLog(const char* tag, uint32_t main_code, uint32_t sub_code)
{
  cobotta::Message message(main_code, sub_code);
  std::stringstream ss;
  ss << "[Lv" << message.getErrorLevel();
  ss << ":" << std::hex << std::uppercase << std::setfill('0') << std::setw(8) << main_code;
  ss << "/" << std::hex << std::uppercase << std::setfill('0') << std::setw(8) << sub_code;
  ss << "] ";
  ss << "<" << tag << "> ";
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
 * Make motor to start or stop.
 * $ rosservice call /cobotta/set_motor_state "{true}"
 * @param req true:motor start false:motor stop
 * @param res true:success false:failure
 * @return true
 */
bool DensoCobottaDriver::setMotorStateSv(SetMotorState::Request& req, SetMotorState::Response& res)
{
  res.success = true;
  try
  {
    if (req.state)
    {
      cobotta_->getMotor()->start();
    }
    else
    {
      cobotta_->getMotor()->stop();
    }
  }
  catch (const CobottaException& e)
  {
    Message::putRosConsole(nullptr, e);
    res.success = false;
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    res.success = false;
  }

  return true;
}
/**
 * Get the state of motor.
 * rosservice call /cobotta/get_motor_state
 * @param res true:motor running false:motor stop
 * @return true
 */
bool DensoCobottaDriver::getMotorStateSv(GetMotorState::Request& /* req */, GetMotorState::Response& res)
{
  bool motor_on;
  res.success = true;
  res.state = true;

  try
  {
    res.state = cobotta_->getMotor()->isRunning();
  }
  catch (const CobottaException& e)
  {
    Message::putRosConsole(nullptr, e);
    res.success = false;
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    res.success = false;
  }

  return true;
}

/**
 * Make brakes to lock or unlock.
 * $ rosservice call /cobotta/set_brake_state
 * @param req "{state: [bool, bool, bool. bool, bool, bool]}" true:lock false:unlock
 * @param res true:success false:failure
 * @return
 */
bool DensoCobottaDriver::setBrakeStateSv(SetBrakeState::Request& req, SetBrakeState::Response& res)
{
  res.success = true;

  if (req.state.size() != CONTROL_JOINT_MAX)
  {
    ROS_ERROR("Failed to set_brake_state. "
              "The nubmer of joint is invalid. (size=%ld)",
              req.state.size());
    res.success = false;
    return true;
  }

  std::array<int, JOINT_MAX> state;
  state.fill(SRV_BRAKE_NONE);
  for (int i = 0; i < CONTROL_JOINT_MAX; i++)
  {
    if (req.state[i])
    {
      state[i] = SRV_BRAKE_LOCK;
    }
    else
    {
      state[i] = SRV_BRAKE_RELEASE;
    }
  }

  try
  {
    cobotta_->getBrake()->change(0, state);
  }
  catch (const CobottaException& e)
  {
    Message::putRosConsole(nullptr, e);
    res.success = false;
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    res.success = false;
  }

  return true;
}

/**
 * Get the state of brakes.
 * $ rosservice call /cobotta/get_brake_state
 * @param res true:success false:failure
 * @return
 */
bool DensoCobottaDriver::getBrakeStateSv(GetBrakeState::Request& /* req */, GetBrakeState::Response& res)
{
  res.success = true;

  try
  {
    auto state = cobotta_->getBrake()->getArmState(0);
    res.state.resize(CONTROL_JOINT_MAX);
    for (int i = 0; i < CONTROL_JOINT_MAX; i++)
    {
      res.state[i] = state[i] == SRV_BRAKE_LOCK ? true : false;
    }
  }
  catch (const CobottaException& e)
  {
    Message::putRosConsole(nullptr, e);
    res.success = false;
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    res.success = false;
  }

  return true;
}

/**
 * Make robot & safety errors to clear except for fatal errors.
 * $ rosservice call /cobotta/clear_error
 * @param res true:success false:failure
 * @return
 */
bool DensoCobottaDriver::clearErrorSv(ClearError::Request& /* req */, ClearError::Response& res)
{
  res.success = true;
  try
  {
    if (cobotta_->getMotor()->isRunning())
      return true;

    cobotta_->getDriver()->clearError();
    cobotta_->getSafetyMcu()->moveToNormal();
  }
  catch (const CobottaException& e)
  {
    Message::putRosConsole(nullptr, e);
    res.success = false;
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    res.success = false;
  }

  return true;
}

/**
 * Make robot errors to clear except for fatal errors.
 * $ rosservice call /cobotta/clear_robot_error
 * @param res true:success false:failure
 * @return
 */
bool DensoCobottaDriver::clearRobotErrorSv(ClearError::Request& /* req */, ClearError::Response& res)
{
  res.success = true;
  try
  {
    cobotta_->getDriver()->clearError();
  }
  catch (const CobottaException& e)
  {
    Message::putRosConsole(nullptr, e);
    res.success = false;
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    res.success = false;
  }

  return true;
}

/**
 * Make state of SafetyMCU to clear errors & move normal except for fatal errors.
 * $ rosservice call /cobotta/clear_safe_state
 * @param res true:success false:failure
 * @return
 */
bool DensoCobottaDriver::clearSafeStateSv(ClearError::Request& /* req */, ClearError::Response& res)
{
  res.success = true;
  try
  {
    cobotta_->getSafetyMcu()->moveToNormal();
  }
  catch (const CobottaException& e)
  {
    Message::putRosConsole(nullptr, e);
    res.success = false;
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    res.success = false;
  }

  return true;
}

/**
 * Make the color of LED to change.
 * $ rosservice call /cobotta/set_LED_state "{red:255, green:255, blue:255, blink_rate:255}"
 * @param res true:success false:failure
 * @return
 */
bool DensoCobottaDriver::setLedStateSv(SetLEDState::Request& req, SetLEDState::Response& res)
{
  res.success = true;

  try
  {
    res.success = cobotta_->getLed()->change(req.blink_rate, req.red, req.green, req.blue);
  }
  catch (const CobottaException& e)
  {
    Message::putRosConsole(nullptr, e);
    res.success = false;
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    res.success = false;
  }

  return true;
}

/**
 * Make the value of mini-Output to send.
 * $ rostopic pub /cobotta/miniIO_output std_msgs/Uint16 "data: 0"
 * @param res true:success false:failure
 * @return
 */
void DensoCobottaDriver::miniIoOutputCallback(const std_msgs::UInt16::ConstPtr& msg)
{
  try
  {
    cobotta_->getMiniIo()->sendOutputStateValue(msg->data);
  }
  catch (const CobottaException& e)
  {
    Message::putRosConsole(nullptr, e);
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
}

const std::shared_ptr<cobotta::Cobotta>& DensoCobottaDriver::getCobotta() const
{
  return cobotta_;
}

bool DensoCobottaDriver::isForceClearFlag() const
{
  return force_clear_flag_;
}

void DensoCobottaDriver::setForceClearFlag(bool forceClearFlag = false)
{
  force_clear_flag_ = forceClearFlag;
}

} // namespace denso_cobotta_driver
