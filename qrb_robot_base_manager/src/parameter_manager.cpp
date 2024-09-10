// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_robot_base_manager/parameter_manager.hpp"

#include <algorithm>
#include <iostream>
#include <iterator>
#include <memory>
#include <mutex>

#include "qrb_robot_base_manager/qrc_utils.hpp"

namespace qrb
{
namespace robot_base_manager
{
ParameterManager::ParameterManager()
{
  pipe_ = qrc_get_pipe(CONFIG_PIPE);
  if (pipe_ == nullptr) {
    std::cerr << "get qrc pipe: " << CONFIG_PIPE << " failed" << std::endl;
    return;
  }
  qrc_register_message_cb(pipe_, &ParameterManager::qrc_message_handle);
  sleep(3);
}

ParameterManager & ParameterManager::get_instance()
{
  static ParameterManager instance;
  return instance;
}

void ParameterManager::qrc_message_handle(struct qrc_pipe_s *, void * data, std::size_t, bool)
{
  std::unique_lock<std::mutex> lock(ParameterManager::get_instance().config_msg_mtx_);

  auto message = static_cast<struct config_msg_s *>(data);
  switch (message->type) {
    case config_msg_type_e::APPLY:
      ParameterManager::get_instance().apply_msg_received_ = true;
      message->data.apply.error_type = message->data.apply.error_type;
      ParameterManager::get_instance().apply_result_ = message->data.apply;

      std::cout << "ParameterManager: got params apply"
                << ", status: " << message->data.apply.status
                << ", error_type: " << message->data.apply.error_type << std::endl;
      break;
    case config_msg_type_e::CAR:
      std::cout << "ParameterManager: car params send success" << std::endl;
      break;
    case config_msg_type_e::MOTION:
      std::cout << "ParameterManager: motion params send success" << std::endl;
      break;
    case config_msg_type_e::SENSOR:
      std::cout << "ParameterManager: sensor params send success" << std::endl;
      break;
    case config_msg_type_e::SCALE:
      std::cout << "ParameterManager: scale params send success" << std::endl;
      break;
    case config_msg_type_e::RC:
      std::cout << "ParameterManager: rc params send success" << std::endl;
      break;
    case config_msg_type_e::OBSTACLE_AVOIDANCE:
      std::cout << "ParameterManager: obstacle avoidance params send success" << std::endl;
      break;
    default:
      std::cerr << "ParameterManager: qrc message not valid, type: " << message->type << std::endl;
      QrcUtils::dump_message(data, sizeof(config_msg_s));
      break;
  }
  lock.unlock();
  ParameterManager::get_instance().config_msg_cond_.notify_all();
}

bool ParameterManager::set_car_parameter(const CarParameter & param)
{
  struct config_car_s config = {};
  config.car_model = car_model_e(param.model);
  config.kinematic_model = kinematic_model_e(param.kinematic);
  config.wheel_space = param.wheel_space;
  config.wheel_perimeter = param.wheel_perimeter;
  struct config_msg_s msg = {};
  msg.type = config_msg_type_e::CAR;
  msg.data.car = config;
  return QrcUtils::send_message(pipe_, &msg, sizeof(msg));
}

bool ParameterManager::set_motion_parameter(const MotionParameter & param)
{
  struct config_motion_s config = {};
  config.max_speed = param.max_speed;
  config.max_angle_speed = param.max_angle_speed;
  config.max_position_dist = param.max_position_dist;
  config.max_position_angle = param.max_position_angle;
  config.max_position_line_speed = param.max_position_line_speed;
  config.max_position_angle_speed = param.max_position_angle_speed;
  std::copy(std::begin(param.pid_speed), std::end(param.pid_speed), std::begin(config.pid_speed));
  std::copy(std::begin(param.pid_position), std::end(param.pid_position),
      std::begin(config.pid_position));
  config.odom_frequency = param.odom_frequency;
  struct config_msg_s msg = {};
  msg.type = config_msg_type_e::MOTION;
  msg.data.motion = config;
  return QrcUtils::send_message(pipe_, &msg, sizeof(msg));
}

bool ParameterManager::set_scale_parameter(const ScaleParameter & param)
{
  struct config_scale_s config
  {
    { 1.0, 1.0 }, { 1.0, 1.0 }, { 1.0, 1.0 }, { 1.0, 1.0 },
  };
  std::copy(std::begin(param.speed), std::end(param.speed), std::begin(config.speed_scale));
  std::copy(
      std::begin(param.position), std::end(param.position), std::begin(config.position_scale));
  std::copy(std::begin(param.speed_odom), std::end(param.speed_odom),
      std::begin(config.speed_odom_scale));
  std::copy(std::begin(param.position_odom), std::end(param.position_odom),
      std::begin(config.position_odom_scale));

  struct config_msg_s msg = {};
  msg.type = config_msg_type_e::SCALE;
  msg.data.scale = config;
  return QrcUtils::send_message(pipe_, &msg, sizeof(msg));
}

bool ParameterManager::set_sensor_parameter(const SensorParameter & param)
{
  struct config_sensor_s config = {};
  config.imu_enable = param.imu_enable;
  config.ultra_enable = param.ultra_enable;
  config.ultra_quantity = param.ultra_quantity;
  struct config_msg_s msg = {};
  msg.type = config_msg_type_e::SENSOR;
  msg.data.sensor = config;
  return QrcUtils::send_message(pipe_, &msg, sizeof(msg));
}

bool ParameterManager::set_remote_controller_parameter(const RemoteControllerParameter & param)
{
  struct config_remote_controller_s config = {};
  config.rc_enable = param.rc_enable;
  config.max_speed = param.max_speed;
  config.max_angle_speed = param.max_angle_speed;
  struct config_msg_s msg = {};
  msg.type = config_msg_type_e::RC;
  msg.data.rc = config;
  return QrcUtils::send_message(pipe_, &msg, sizeof(msg));
}

bool ParameterManager::set_obstacle_avoidance_parameter(const ObstacleAvoidanceParameter & param)
{
  struct config_obstacle_avoidance_s config = {};
  config.bottom_dist = param.bottom_dist;
  config.side_dist = param.side_dist;
  config.front_dist = param.front_dist;
  struct config_msg_s msg = {};
  msg.type = config_msg_type_e::OBSTACLE_AVOIDANCE;
  msg.data.ob = config;
  return QrcUtils::send_message(pipe_, &msg, sizeof(msg));
}

bool ParameterManager::apply_parameters()
{
  struct config_apply_s config = {};
  struct config_msg_s msg = {};
  msg.type = config_msg_type_e::APPLY;
  msg.data.apply = config;
  if (!QrcUtils::send_message(pipe_, &msg, sizeof(msg))) {
    return false;
  }

  {
    std::unique_lock<std::mutex> lock(config_msg_mtx_);
    config_msg_cond_.wait(lock, [this] { return apply_msg_received_; });
    apply_msg_received_ = false;
  }

  if (apply_result_.status != 0) {
    std::cerr << "ParameterManager::apply_parameters failed"
              << ", task_id = " << apply_result_.error_type << std::endl;
    return false;
  }
  return true;
}

}  // namespace robot_base_manager
}  // namespace qrb
