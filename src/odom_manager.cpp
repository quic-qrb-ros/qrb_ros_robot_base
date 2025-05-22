// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_robot_base_manager/odom_manager.hpp"

#include <iostream>
#include <memory>
#include <utility>

#include "motion_msg.h"
#include "qrb_robot_base_manager/qrc_utils.hpp"

namespace qrb
{
namespace robot_base_manager
{
OdomManager::OdomManager()
{
  pipe_ = qrc_get_pipe(ODOM_PIPE);
  if (pipe_ == nullptr) {
    std::cerr << "get qrc pipe: " << ODOM_PIPE << " failed" << std::endl;
    return;
  }
  auto callback = [](struct qrc_pipe_s *, void * data, std::size_t, bool) {
    qrc_message_handle(data);
  };
  qrc_register_message_cb(pipe_, callback);
}

OdomManager & OdomManager::get_instance()
{
  static OdomManager instance;
  return instance;
}

void OdomManager::qrc_message_handle(void * data)
{
  std::unique_lock<std::mutex> lock(OdomManager::get_instance().odom_msg_mtx_);
  auto msg = static_cast<struct motion_odom_s *>(data);
  timespec timestamp{};
  timestamp.tv_sec = msg->sec;
  timestamp.tv_nsec = msg->ns;
  switch (msg->type) {
    case odom_msg_type_e::ODOM_SPEED:
      if (get_instance().speed_cb_ != nullptr) {
        get_instance().speed_cb_(msg->x, msg->z, timestamp);
      }
      break;
    case odom_msg_type_e::ODOM_POSITION:
      std::cout << "OdomManager: got position odom success" << std::endl;
      if (get_instance().position_cb_ != nullptr) {
        get_instance().position_cb_(msg->x, msg->z, timestamp);
      }
      break;
    default:
      std::cerr << "OdomManager: qrc message not valid, type: " << msg->type << std::endl;
      QrcUtils::dump_message(data, sizeof(motion_odom_s));
      break;
  }
}

void OdomManager::register_speed_callback(std::function<void(float, float, timespec)> cb)
{
  speed_cb_ = std::move(cb);
}

void OdomManager::register_position_callback(std::function<void(float, float, timespec)> cb)
{
  position_cb_ = std::move(cb);
}

}  // namespace robot_base_manager
}  // namespace qrb
