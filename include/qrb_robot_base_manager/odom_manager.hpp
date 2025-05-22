// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROBOT_BASE_MANAGER__ODOM_MANAGER_HPP_
#define QRB_ROBOT_BASE_MANAGER__ODOM_MANAGER_HPP_

#include <functional>
#include <memory>
#include <mutex>

struct qrc_pipe_s;

namespace qrb
{
namespace robot_base_manager
{
class OdomManager
{
private:
  OdomManager();

public:
  OdomManager(const OdomManager &) = delete;
  OdomManager & operator=(const OdomManager &) = delete;
  OdomManager(const OdomManager &&) = delete;
  OdomManager & operator=(const OdomManager &&) = delete;

  static OdomManager & get_instance();

  void register_speed_callback(std::function<void(float, float, timespec)>);
  void register_position_callback(std::function<void(float, float, timespec)>);

private:
  static void qrc_message_handle(void * data);

  struct qrc_pipe_s * pipe_ = nullptr;
  std::function<void(float, float, timespec)> speed_cb_;
  std::function<void(float, float, timespec)> position_cb_;
  std::mutex odom_msg_mtx_;
};

}  // namespace robot_base_manager
}  // namespace qrb

#endif  // QRB_ROBOT_BASE_MANAGER__ODOM_MANAGER_HPP_
