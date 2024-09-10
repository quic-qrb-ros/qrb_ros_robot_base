// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROBOT_BASE_MANAGER__PARAMETER_MANAGER_HPP_
#define QRB_ROBOT_BASE_MANAGER__PARAMETER_MANAGER_HPP_

#include <condition_variable>
#include <memory>

#include "config_msg.h"
#include "qrc_msg_management.h"

namespace qrb
{
namespace robot_base_manager
{
enum class CarModel
{
  CYCLE_CAR,
  AMR_6040,
  CAR_MODE_MAX,
};

enum KinematicModel
{
  DIFF_CAR,
  ACKERMAN_CAR,
  KINEMATIC_MODEL_MAX,
};

struct CarParameter
{
  CarModel model;
  KinematicModel kinematic;
  float wheel_space;
  float wheel_perimeter;
};

struct MotionParameter
{
  float max_speed;
  float max_angle_speed;
  float max_position_dist;
  float max_position_angle;
  float max_position_line_speed;
  float max_position_angle_speed;
  float pid_speed[3];
  float pid_position[3];
  uint32_t odom_frequency;
};

struct ScaleParameter
{
  float speed[2];
  float position[2];
  float speed_odom[2];
  float position_odom[2];
};

struct SensorParameter
{
  uint8_t imu_enable;
  uint8_t ultra_enable;
  uint8_t ultra_quantity;
};

struct RemoteControllerParameter
{
  uint8_t rc_enable;
  float max_speed;
  float max_angle_speed;
};

struct ObstacleAvoidanceParameter
{
  float bottom_dist;  // unit: m
  float side_dist;
  float front_dist;
};

struct TimeParameter
{
  bool test_transport_latency_enable;
  unsigned int time_sync_interval_sec;
  unsigned int time_sync_threshold_ms;
};

struct RobotBaseParameter
{
  CarParameter car_param;
  MotionParameter motion_param;
  SensorParameter sensor_param;
  ScaleParameter scale_param;
  RemoteControllerParameter rc_param;
  ObstacleAvoidanceParameter oba_param;
  TimeParameter time_param;
};

class ParameterManager
{
private:
  ParameterManager();

public:
  ParameterManager(const ParameterManager &) = delete;
  ParameterManager & operator=(const ParameterManager &) = delete;
  ParameterManager(const ParameterManager &&) = delete;
  ParameterManager & operator=(const ParameterManager &&) = delete;

  static ParameterManager & get_instance();

  bool set_car_parameter(const CarParameter & param);
  bool set_motion_parameter(const MotionParameter & param);
  bool set_scale_parameter(const ScaleParameter & param);
  bool set_sensor_parameter(const SensorParameter & param);
  bool set_remote_controller_parameter(const RemoteControllerParameter & param);
  bool set_obstacle_avoidance_parameter(const ObstacleAvoidanceParameter & param);
  bool apply_parameters();

private:
  static void qrc_message_handle(struct qrc_pipe_s * pipe, void * data, std::size_t len, bool resp);

  struct qrc_pipe_s * pipe_ = nullptr;
  std::mutex config_msg_mtx_;
  std::condition_variable config_msg_cond_;
  bool apply_msg_received_ = false;

  struct config_apply_s apply_result_;
};

}  // namespace robot_base_manager
}  // namespace qrb

#endif  // QRB_ROBOT_BASE_MANAGER__PARAMETER_MANAGER_HPP_
