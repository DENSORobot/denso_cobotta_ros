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
#include <fstream>

#include <yaml-cpp/yaml.h>
#include "denso_cobotta_driver/denso_cobotta_driver.h"
#include "denso_cobotta_lib/cobotta.h"
#include "denso_cobotta_lib/gripper.h"

void signalHandler(int sig)
{
  int fd;
  fd = open(cobotta_common::PATH_DEVFILE, O_RDWR);
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
    ros::Rate rate(1.0 / cobotta_common::DRIVER_UPDATE_PERIOD);
    while (ros::ok())
    {
      driver.update();
      rate.sleep();
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

    // Read rang value
    bool enable_calset = false;
    if (!nh.getParam("enable_calset", enable_calset))
    {
      ROS_WARN("Failed to get param 'enable_calset'.");
    }

    read_rang_value_ = false;
    if (enable_calset)
    {
      if (nh.getParam("rang_value", rang_value_))
      {
        if (rang_value_.size() == CONTROL_JOINT_MAX)
        {
          read_rang_value_ = true;
        }
        else
        {
          ROS_WARN("Invalid 'rang_value' length.");
        }
      }
      else
      {
        ROS_WARN("Failed to get param 'rang_value'.");
      }
    }

    cobotta_->getLed()->forceChange(static_cast<uint32_t>(LedColorTable::White));

    // Service server
    sv_set_motor_ = nh.advertiseService("set_motor_state", &DensoCobottaDriver::setMotorStateSv, this);
    sv_get_motor_ = nh.advertiseService("get_motor_state", &DensoCobottaDriver::getMotorStateSv, this);
    sv_set_brake_ = nh.advertiseService("set_brake_state", &DensoCobottaDriver::setBrakeStateSv, this);
    sv_get_brake_ = nh.advertiseService("get_brake_state", &DensoCobottaDriver::getBrakeStateSv, this);
    if (read_rang_value_)
    {
      sv_exec_calset_ = nh.advertiseService("exec_calset", &DensoCobottaDriver::execCalsetSv, this);
    }
    sv_clear_error_ = nh.advertiseService("clear_error", &DensoCobottaDriver::clearErrorSv, this);
    sv_clear_robot_error_ = nh.advertiseService("clear_robot_error", &DensoCobottaDriver::clearRobotErrorSv, this);
    sv_clear_safe_state_ = nh.advertiseService("clear_safe_state", &DensoCobottaDriver::clearSafeStateSv, this);
    sv_set_led_ = nh.advertiseService("set_LED_state", &DensoCobottaDriver::setLedStateSv, this);

    // Publisher
    pub_function_button_ = nh.advertise<std_msgs::Bool>("function_button", 1);
    pub_plus_button_ = nh.advertise<std_msgs::Bool>("plus_button", 1);
    pub_minus_button_ = nh.advertise<std_msgs::Bool>("minus_button", 1);
    pub_mini_io_input_ = nh.advertise<std_msgs::UInt16>("miniIO_input", 1);
    pub_robot_state_ = nh.advertise<denso_cobotta_driver::RobotState>("robot_state", 64);
    pub_safe_state_ = nh.advertise<denso_cobotta_driver::SafeState>("safe_state", 64);
    pub_gripper_state_ = nh.advertise<std_msgs::Bool>("gripper_state", 1);

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
        struct StateCode sc = this->getCobotta()->getDriver()->dequeue(i);
        std::string tag = "robot#" + std::to_string(i);
        this->putRosLog(tag.c_str(), sc.main_code, sc.sub_code);

        RobotState pub;
        pub.arm_no = i;
        pub.state_code = sc.main_code;
        pub.state_subcode = sc.sub_code;
        this->pub_robot_state_.publish(pub);

        auto msg = Message::getMessageInfo(sc.main_code);
        if (msg.level >= 4)
          lv4_error = true;
      }
    }
    /* safetyMCU */
    for (int i = 0; i < info.getSafetyMcuQueueSize(); i++)
    {
      struct StateCode sc = this->getCobotta()->getSafetyMcu()->dequeue();
      this->putRosLog("safety", sc.main_code, sc.sub_code);

      SafeState pub;
      pub.state_code = sc.main_code;
      pub.state_subcode = sc.sub_code;
      this->pub_safe_state_.publish(pub);

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
    std_msgs::Bool function_button_state;
    std_msgs::Bool plus_button_state;
    std_msgs::Bool minus_button_state;
    std_msgs::UInt16 mini_io_input;
    std_msgs::Bool gripper_state;

    function_button_state.data = info.isFunctionButton();
    this->pub_function_button_.publish(function_button_state);
    plus_button_state.data = info.isPlusButton();
    this->pub_plus_button_.publish(plus_button_state);
    minus_button_state.data = info.isMinusButton();
    this->pub_minus_button_.publish(minus_button_state);
    mini_io_input.data = info.getMiniIo();
    this->pub_mini_io_input_.publish(mini_io_input);
    gripper_state.data = info.isGripperState();
    this->pub_gripper_state_.publish(gripper_state);
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
    ROS_ERROR("Failed to run 'set_brake_state'. "
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
 * Save the parameters used for improving accuracy of the pose.
 *
 * @param res true:success false:failure
 * @return
 */
bool DensoCobottaDriver::execCalsetSv(ExecCalset::Request& /* req */, ExecCalset::Response& res)
{
  res.success = true;
  static const double process_wait_time = 0.02;
  static const double delta_angle = 15;
  if (!read_rang_value_)
  {
    ROS_ERROR("Rang value is not set.");
    res.success = false;
    return true;
  }
  for (int i = 0; i < CONTROL_JOINT_MAX; i++)
  {
    double tmp_degree = cobotta_common::CALSET_POSE[i] > 0 ? rang_value_[i] - cobotta_common::CALSET_POSE[i] :
                                                             cobotta_common::CALSET_POSE[i] - rang_value_[i];
    if (tmp_degree < 0 || tmp_degree > delta_angle)
    {
      ROS_ERROR("Rang value differs greatly from mecha end.");
      res.success = false;
      return true;
    }
  }

  auto motor = [&](bool on) {
    SetMotorState::Request motor_req;
    SetMotorState::Response motor_res;
    motor_req.state = on;
    setMotorStateSv(motor_req, motor_res);
    return (bool)(motor_res.success);
  };

  auto failed_process = [&]() {
    res.success = false;
    motor(false);
    ros::Duration(process_wait_time).sleep();  // wait for stopping
    setDeviationParameters(POSITION_DEVIATION_PARAMS);
  };

  std::vector<int32_t> pulse_offset;
  pulse_offset = { 0, 0, 0, 0, 0, 0 };
  // ++++++++++++++++++++Begin AutoCal++++++++++++++++++++
  auto deviation_params = POSITION_DEVIATION_PARAMS;
  for (int i = 0; i < CONTROL_JOINT_MAX; i++)
  {
    deviation_params[i] = INT16_MAX;
  }

  motor(false);
  ros::Duration(process_wait_time).sleep();       // wait for stopping
  if (!setDeviationParameters(deviation_params))  // send parameters
  {
    failed_process();
    return true;
  }
  ros::Duration(process_wait_time).sleep();  // wait for sending parameters
  if (!motor(true))
  {
    failed_process();
    return true;
  }
  ros::Duration(process_wait_time).sleep();  // wait for starting

  // move to start position
  MoveParam move_param;
  auto set_start_move_param = [&]() {
    move_param.max_acceleration = ARM_MAX_ACCELERATION;
    move_param.max_velocity = ARM_MAX_VELOCITY;
    for (int i = 0; i < CONTROL_JOINT_MAX; i++)
    {
      double tmp_degree = cobotta_common::CALSET_POSE[i];
      move_param.target_position[i] = tmp_degree * M_PI / 180.0;
      move_param.current_offset[i] = 0;
      move_param.current_limit[i] = 0x0FFF;
    }
  };
  set_start_move_param();
  if (!sineMove(move_param))
  {
    failed_process();
    return true;
  }
  ros::Duration(0.1).sleep();  // wait to finish moving
  // prepare to move not to cantact with mecha end
  ROS_INFO("Moving to the mecha end .... Please wait for about 60 seconds.");
  for (int i = 0; i < CONTROL_JOINT_MAX; i++)
  {
    double tmp_degree = cobotta_common::CALSET_POSE[i] > 0 ? cobotta_common::CALSET_POSE[i] + delta_angle :
                                                             cobotta_common::CALSET_POSE[i] - delta_angle;
    move_param.max_velocity[i] = 0.02;
    move_param.target_position[i] = tmp_degree * M_PI / 180.0;
    move_param.current_limit[i] = 0x04FF;
  }
  if (!sineMove(move_param))
  {
    failed_process();
    return true;
  }
  ros::Duration(0.1).sleep();  // wait to finish moving
  // ++++++++++++++++++++End AutoCal++++++++++++++++++++

  std::array<int32_t, JOINT_MAX> cur_pulse;  // [pulse]
  getPulse(0, cur_pulse);                    // get current pulse
  motor(false);
  ros::Duration(process_wait_time).sleep();  // wait for starting

  // caluculate pulse offset to adjust current pulse
  for (int i = 0; i < CONTROL_JOINT_MAX; i++)
  {
    pulse_offset[i] = ARM_COEFF_OUTPOS_TO_PULSE[i] * rang_value_[i] * M_PI / 180.0 - cur_pulse[i];
    if (std::abs(pulse_offset[i] / ARM_COEFF_OUTPOS_TO_PULSE[i]) > OFFSET_LIMIT[i])
    {
      ROS_ERROR("Rang value differs greatly from encoder value.");
      failed_process();
      return true;
    }
  }
  // save pulse offset
  try
  {
    // create params file
    {
      std::ifstream ifs(cobotta_common::TEMP_PARAMS_PATH);
      if (!ifs)
      {
        std::ofstream ofs(cobotta_common::TEMP_PARAMS_PATH);
      }
    }
    YAML::Node cobotta_params = YAML::LoadFile(cobotta_common::TEMP_PARAMS_PATH);
    std::ofstream ofs(cobotta_common::TEMP_PARAMS_PATH);
    cobotta_params["pulse_offset"] = pulse_offset;
    ofs << cobotta_params << std::endl;
  }
  catch (std::exception& e)
  {
    ROS_ERROR("Failed to save the CALSET data.");
    failed_process();
    return true;
  }

  setDeviationParameters(POSITION_DEVIATION_PARAMS);
  ROS_INFO("Success to save the CALSET data.");
  ROS_INFO("Pulse offset is [J1: %i, J2: %i, J3: %i, J4: %i, J5: %i, J6: %i].", pulse_offset[0], pulse_offset[1],
           pulse_offset[2], pulse_offset[3], pulse_offset[4], pulse_offset[5]);

  // Move to the valid pose of Moveit.
  motor(true);
  ros::Duration(process_wait_time).sleep();  // wait for starting
  set_start_move_param();
  for (int i = 0; i < CONTROL_JOINT_MAX; i++)
  {
    move_param.target_position[i] = cobotta_common::HOME_POSE[i] * M_PI / 180.0;
  }
  if (!sineMove(move_param))
  {
    ROS_WARN("Failed to move home pose.");
  }
  ROS_INFO("Success to move home pose.");
  return true;
}

/**
 * Set deviation parameters
 *
 * @param values Parameters to send
 * @return true:success false:failure
 */
bool DensoCobottaDriver::setDeviationParameters(const std::array<uint16_t, JOINT_MAX + 1>& values)
{
  bool success = true;
  try
  {
    std::array<uint16_t, JOINT_MAX + 1> recv_values;
    Driver::writeHwAcyclicCommAll(cobotta_->getFd(), POSITION_DEVIATION_ADDRESS | ACYCLIC_WRITE_MASK, values,
                                  recv_values);
  }
  catch (const CobottaException& e)
  {
    Message::putRosConsole(nullptr, e);
    success = false;
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    success = false;
  }
  return success;
}

/**
 * Get encoder values
 *
 * @param arm_no
 * @param pulse Encoder values
 * @return true:success false:failure
 */
bool DensoCobottaDriver::getPulse(long arm_no, std::array<int32_t, JOINT_MAX>& pulse)
{
  SRV_COMM_RECV recv_data{ 0 };

  try
  {
    recv_data = Driver::readHwEncoder(cobotta_->getFd(), arm_no);
  }
  catch (const CobottaException& e)
  {
    ROS_ERROR_STREAM(e.what());
    return false;
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    return false;
  }

  for (int i = 0; i < pulse.size(); i++)
  {
    pulse[i] = recv_data.encoder[i];
  }
  return true;
}

/**
 * Move robot arm. The speed follows sine curve.
 *
 * @param move_param The parameter of the move.
 * @return true:success false:failure
 */
bool DensoCobottaDriver::sineMove(const MoveParam& move_param)
{
  std::array<int32_t, JOINT_MAX> cur_pulse;              // [pulse]
  std::array<double, CONTROL_JOINT_MAX> start_pos_rad;   // [rad]
  std::array<double, CONTROL_JOINT_MAX> rotation_angle;  // [rad]
  std::array<double, CONTROL_JOINT_MAX> velocity;        // [rad/s]
  int32_t max_count;

  auto is_motor_runnning = [&]() {
    bool is_running = cobotta_->getMotor()->isRunning();
    if (!is_running)
    {
      ROS_WARN("Motor is not running.");
    }
    return is_running;
  };

  // Get current position
  if (is_motor_runnning() && getPulse(0, cur_pulse))
  {
    for (int i = 0; i < CONTROL_JOINT_MAX; i++)
    {
      start_pos_rad[i] = cur_pulse[i] / ARM_COEFF_OUTPOS_TO_PULSE[i];
    }
  }
  else
  {
    return false;
  }
  // Caluculate rotation angle
  for (int i = 0; i < CONTROL_JOINT_MAX; i++)
  {
    rotation_angle[i] = move_param.target_position[i] - start_pos_rad[i];
  }
  if (!calculateVelocity(move_param, rotation_angle, velocity, max_count))
  {
    return false;
  }

  // Prepare to move
  SRV_COMM_SEND send_data;
  send_data.arm_no = 0;
  send_data.discontinuous = 0;
  send_data.disable_cur_lim = 0;
  send_data.stay_here = 0;
  for (int i = 0; i < CONTROL_JOINT_MAX; i++)
  {
    send_data.current_limit[i] = move_param.current_limit[i];
    send_data.current_offset[i] = move_param.current_offset[i];
  }

  double move_time = max_count * SERVO_PERIOD;
  ros::Time nowTime = ros::Time::now(), checkTime = nowTime, updateStartTime = nowTime;
  // Loop to send Position command
  const static int wait_count = 125;
  for (int count = 0; count < wait_count; count++)
  {
    for (int i = 0; i < CONTROL_JOINT_MAX; i++)
    {
      send_data.position[i] = ARM_COEFF_OUTPOS_TO_PULSE[i] * start_pos_rad[i];
    }
    if (!(is_motor_runnning() && setServoUpdateData(send_data)))
    {
      return false;
    }
  }
  for (int count = 1; count <= max_count; count++)
  {
    for (int i = 0; i < CONTROL_JOINT_MAX; i++)
    {
      // Calculate position
      send_data.position[i] =
          ARM_COEFF_OUTPOS_TO_PULSE[i] *
          (start_pos_rad[i] + (velocity[i] * move_time / M_PI) * (1 - cos(M_PI * count / (double)max_count)));
    }

    if (count == max_count)
    {
      send_data.stay_here = 1;
    }
    if (!(is_motor_runnning() && setServoUpdateData(send_data)))
    {
      return false;
    }
  }
  return true;
}

/**
 * Decide max velocity of sine curve to be within acceleration and velocity limit.
 *
 * @param move_param The parameter of the move.
 * @return true:success false:failure
 */
bool DensoCobottaDriver::calculateVelocity(const MoveParam& move_param,
                                           const std::array<double, CONTROL_JOINT_MAX>& rotation_angle,
                                           std::array<double, CONTROL_JOINT_MAX>& velocity, int32_t& max_count)
{
  // Calculate moving time [s]
  double move_time = 0;
  for (int i = 0; i < CONTROL_JOINT_MAX; i++)
  {
    // Calculate velocity
    double tmp_velocity = std::sqrt(move_param.max_acceleration[i] * std::abs(rotation_angle[i]) *
                                    0.5);  // tmp_velocity is the velocity based on move_param.max_acceleration
    velocity[i] =
        tmp_velocity < move_param.max_velocity[i] ? tmp_velocity : move_param.max_velocity[i];  // velocity is lower one

    double tmpTime = velocity[i] < 0.000000000001 ? 0 : M_PI * std::abs(rotation_angle[i]) * 0.5 / velocity[i];
    // Based on the one with the longest move time
    move_time = move_time > tmpTime ? move_time : tmpTime;
  }
  if (move_time == 0)
  {
    return false;
  }
  // Calculate move velocity [rad/s]
  max_count = std::ceil(move_time / SERVO_PERIOD);  // round up
  move_time = max_count * SERVO_PERIOD;
  for (int i = 0; i < CONTROL_JOINT_MAX; i++)
  {
    velocity[i] = (rotation_angle[i]) * 0.5 * M_PI / move_time;
  }
  return true;
}

bool DensoCobottaDriver::setServoUpdateData(const SRV_COMM_SEND& send_data)
{
  try
  {
    struct DriverCommandInfo info = Driver::writeHwUpdate(cobotta_->getFd(), send_data);
    if (info.result == 0x0F408101)
    {
      // The current number of commands in buffer is 11.
      // To avoid buffer overflow, sleep 8 msec
      ros::Duration(cobotta_common::getPeriod()).sleep();
    }
    else if (info.result == 0x84400502)
    {
      // buffer full
      ROS_WARN("Command buffer overflow...");
      ros::Duration(cobotta_common::getPeriod() * 2).sleep();
    }
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    return false;
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

}  // namespace denso_cobotta_driver
