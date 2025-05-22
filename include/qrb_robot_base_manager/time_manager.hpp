// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROBOT_BASE_MANAGER__TIME_MANAGER_HPP_
#define QRB_ROBOT_BASE_MANAGER__TIME_MANAGER_HPP_

#include <bits/types/struct_timespec.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>

#include <condition_variable>
#include <future>
#include <memory>
#include <ostream>
#include <thread>

#include "qrb_robot_base_manager/parameter_manager.hpp"
#include "qrc_msg_management.h"

namespace qrb
{
namespace robot_base_manager
{
constexpr long NS_PER_S = 1000000000L;
constexpr long NS_PER_MS = 1000000L;

static struct timespec operator+(const timespec & lhs, const timespec & rhs);
static struct timespec operator-(const timespec & lhs, const timespec & rhs);
static std::ostream & operator<<(std::ostream & os, const timespec & ts);

class TimeManager
{
private:
  TimeManager();

public:
  TimeManager(const TimeManager &) = delete;
  TimeManager & operator=(const TimeManager &) = delete;
  TimeManager(const TimeManager &&) = delete;
  TimeManager & operator=(const TimeManager &&) = delete;

  static TimeManager & get_instance();

  void start_time_sync();
  void stop_time_sync();
  void set_time_param(const TimeParameter & time_param);

private:
  static void qrc_message_handle(struct qrc_pipe_s * pipe, void * data, std::size_t len, bool resp);
  struct timespec get_mcb_time();
  struct timespec get_local_time();

  bool test_transport_latency();
  bool need_time_sync();
  bool trigger_time_sync();
  void sync_time_if_need();
  static void time_sync_handler(union sigval sv);

  struct qrc_pipe_s * pipe_ = nullptr;

  struct timespec mcb_time_;
  struct timespec transport_latency_ = { 0, 0 };

  std::mutex time_sync_mtx_;
  std::condition_variable time_loop_cond_;
  bool time_loop_received_ = false;
  bool get_time_received_ = false;
  bool set_time_received_ = false;
  timer_t timer_id_;

  struct TimeParameter time_param_ = { 0, 300, 10 };
};

}  // namespace robot_base_manager
}  // namespace qrb

#endif  // QRB_ROBOT_BASE_MANAGER__TIME_MANAGER_HPP_
