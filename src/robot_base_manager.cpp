// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear
#include "qrb_robot_base_manager/robot_base_manager.hpp"

#include <iostream>

#include "qrb_robot_base_manager/time_manager.hpp"
#include "qrb_robot_base_manager/watchdog.hpp"

namespace qrb
{
namespace robot_base_manager
{
RobotBaseManager::RobotBaseManager() {}

RobotBaseManager & RobotBaseManager::get_instance()
{
  static RobotBaseManager instance;
  return instance;
}

bool RobotBaseManager::init_qrc()
{
  // qrc will reset robot base here
  if (!init_qrc_management()) {
    return false;
  }
  return true;
}

bool RobotBaseManager::deinit_qrc()
{
  // qrc will reset robot base
  return deinit_qrc_management();
}

bool RobotBaseManager::download_parameters(const RobotBaseParameter & robot_base_param)
{
  if (!ParameterManager::get_instance().set_car_parameter(robot_base_param.car_param) ||
      !ParameterManager::get_instance().set_motion_parameter(robot_base_param.motion_param) ||
      !ParameterManager::get_instance().set_sensor_parameter(robot_base_param.sensor_param) ||
      !ParameterManager::get_instance().set_remote_controller_parameter(
          robot_base_param.rc_param) ||
      !ParameterManager::get_instance().set_scale_parameter(robot_base_param.scale_param) ||
      !ParameterManager::get_instance().set_obstacle_avoidance_parameter(
          robot_base_param.oba_param) ||
      !ParameterManager::get_instance().apply_parameters()) {
    return false;
  }
  return true;
}

bool RobotBaseManager::init_robot_base_manager(const RobotBaseParameter & robot_base_param)
{
  if (!init_qrc()) {
    std::cerr << "RobotBaseManager: init qrc failed" << std::endl;
    return false;
  }

  if (!download_parameters(robot_base_param)) {
    std::cerr << "RobotBaseManager: download parameters failed" << std::endl;
    return false;
  }

  TimeManager::get_instance().set_time_param(robot_base_param.time_param);
  TimeManager::get_instance().start_time_sync();

  Watchdog::get_instance().start_watch();
  return true;
}

void RobotBaseManager::release_robot_base_manager()
{
  std::cout << "RobotBaseManager: release robot base manager" << std::endl;
  Watchdog::get_instance().stop_watch();
  deinit_qrc();
  TimeManager::get_instance().stop_time_sync();
}

}  // namespace robot_base_manager
}  // namespace qrb
