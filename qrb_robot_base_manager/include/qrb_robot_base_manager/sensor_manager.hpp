// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROBOT_BASE_MANAGER__SENSOR_MANAGER_HPP_
#define QRB_ROBOT_BASE_MANAGER__SENSOR_MANAGER_HPP_

#include <functional>
#include <memory>

#include "qrc_msg_management.h"

namespace qrb
{
namespace robot_base_manager
{
struct IMU
{
  float accel_x;
  float accel_y;
  float accel_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  struct timespec timestamp;
};

class SensorManager
{
private:
  SensorManager();

public:
  SensorManager(const SensorManager &) = delete;
  SensorManager & operator=(const SensorManager &) = delete;
  SensorManager(const SensorManager &&) = delete;
  SensorManager & operator=(const SensorManager &&) = delete;

  static SensorManager & get_instance();
  void register_imu_callback(std::function<void(const IMU &)> cb);

private:
  static void qrc_message_handle(void * data);

  struct qrc_pipe_s * pipe_ = nullptr;
  std::function<void(const IMU &)> imu_cb_;
};

}  // namespace robot_base_manager
}  // namespace qrb

#endif  // QRB_ROBOT_BASE_MANAGER__SENSOR_MANAGER_HPP_
