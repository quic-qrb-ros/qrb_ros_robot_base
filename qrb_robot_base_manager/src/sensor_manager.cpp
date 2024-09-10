// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_robot_base_manager/sensor_manager.hpp"

#include <iostream>
#include <memory>
#include <mutex>
#include <utility>

#include "imu_msg.h"
#include "qrb_robot_base_manager/qrc_utils.hpp"

namespace qrb
{
namespace robot_base_manager
{
SensorManager::SensorManager()
{
  pipe_ = qrc_get_pipe(IMU_PIPE);
  if (pipe_ == nullptr) {
    std::cerr << "get qrc pipe: " << IMU_PIPE << " failed" << std::endl;
    return;
  }

  auto callback = [](struct qrc_pipe_s *, void * data, std::size_t, bool) {
    qrc_message_handle(data);
  };
  qrc_register_message_cb(pipe_, callback);
}

SensorManager & SensorManager::get_instance()
{
  static SensorManager instance;
  return instance;
}

void SensorManager::qrc_message_handle(void * data)
{
  auto message = static_cast<struct imu_msg_s *>(data);

  IMU imu{};
  imu.accel_x = message->data.xa;
  imu.accel_y = message->data.ya;
  imu.accel_z = message->data.za;
  imu.gyro_x = message->data.xg;
  imu.gyro_y = message->data.yg;
  imu.gyro_z = message->data.zg;
  imu.timestamp.tv_sec = message->sec;
  imu.timestamp.tv_nsec = static_cast<int64_t>(message->ns);

  get_instance().imu_cb_(imu);
}

void SensorManager::register_imu_callback(std::function<void(const IMU &)> cb)
{
  imu_cb_ = std::move(cb);
}

}  // namespace robot_base_manager
}  // namespace qrb