// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROBOT_BASE_MANAGER__WATCHDOG_HPP_
#define QRB_ROBOT_BASE_MANAGER__WATCHDOG_HPP_

#include <chrono>
#include <condition_variable>
#include <memory>
#include <thread>

#include "qrb_robot_base_manager/error.hpp"
#include "qrc_msg_management.h"

namespace qrb
{
namespace robot_base_manager
{
class Watchdog
{
private:
  Watchdog(int timeoutInterval);

public:
  Watchdog(const Watchdog &) = delete;
  Watchdog & operator=(const Watchdog &) = delete;
  Watchdog(const Watchdog &&) = delete;
  Watchdog & operator=(const Watchdog &&) = delete;

  static Watchdog & get_instance(int timeout_interval = 10);
  void start_watch();
  void stop_watch();
  void register_error_callback(std::function<void(const Error &)>);

private:
  static void qrc_message_handle(struct qrc_pipe_s * pipe, void * data, std::size_t len, bool resp);
  void monitor();
  struct qrc_pipe_s * pipe_ = nullptr;
  bool running_ = false;
  int timeout_interval_ = 10;
  bool watchdog_msg_received_ = false;
  std::mutex watchdog_msg_mtx_;
  std::condition_variable watchdog_msg_cond_;
  std::thread monitor_thread_;

  std::function<void(const Error &)> error_cb_{ nullptr };
};

}  // namespace robot_base_manager
}  // namespace qrb

#endif  // QRB_ROBOT_BASE_MANAGER__WATCHDOG_HPP_
