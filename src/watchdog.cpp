// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear
#include "qrb_robot_base_manager/watchdog.hpp"

#include <iostream>

#include "charger_control_msg.h"
#include "client_control_msg.h"
#include "config_msg.h"
#include "emergency_msg.h"
#include "imu_msg.h"
#include "misc_msg.h"
#include "motion_msg.h"
#include "time_sync_msg.h"

namespace qrb
{
namespace robot_base_manager
{
Watchdog::Watchdog(int timeout_interval) : running_(false), timeout_interval_(timeout_interval) {}

Watchdog & Watchdog::get_instance(int timeout_interval)
{
  static Watchdog instance(timeout_interval);
  return instance;
}

void Watchdog::start_watch()
{
  running_ = true;
  pipe_ = qrc_get_pipe(MISC_PIPE);
  if (pipe_ == nullptr) {
    std::cerr << "get qrc pipe: " << CONFIG_PIPE << " failed" << std::endl;
    return;
  }
  qrc_register_message_cb(pipe_, &Watchdog::qrc_message_handle);
  sleep(3);
  monitor_thread_ = std::thread(&Watchdog::monitor, this);
}

void Watchdog::stop_watch()
{
  running_ = false;
  watchdog_msg_received_ = true;
  watchdog_msg_cond_.notify_all();
  if (monitor_thread_.joinable()) {
    monitor_thread_.join();
  }
  std::cout << "Watchdog: stop watchdog" << std::endl;
}

void Watchdog::qrc_message_handle(struct qrc_pipe_s *, void * data, std::size_t, bool)
{
  std::unique_lock<std::mutex> lock(Watchdog::get_instance().watchdog_msg_mtx_);
  auto message = static_cast<struct watchdog_msg *>(data);

  Watchdog::get_instance().watchdog_msg_received_ = true;
  lock.unlock();
  Watchdog::get_instance().watchdog_msg_cond_.notify_all();
}

void Watchdog::monitor()
{
  Error error{};

  while (running_) {
    std::unique_lock<std::mutex> lock(watchdog_msg_mtx_);
    if (watchdog_msg_cond_.wait_for(lock, std::chrono::seconds(timeout_interval_),
            [this] { return watchdog_msg_received_; })) {
      watchdog_msg_received_ = false;
    } else {
      std::cerr << "Watchdog warning: No message received in the last " << timeout_interval_
                << " seconds from robot base." << std::endl;
      error.set_type(static_cast<int>(robot_base_error_e::ERROR_WATCHDOG));
      error.set_message(
          error.get_error_messages(static_cast<int>(robot_base_error_e::ERROR_WATCHDOG)));
      if (error_cb_) {
        error_cb_(error);
      }
    }
  }
}

void Watchdog::register_error_callback(std::function<void(const Error &)> cb)
{
  error_cb_ = std::move(cb);
}

}  // namespace robot_base_manager
}  // namespace qrb
