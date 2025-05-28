// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_robot_base_manager/time_manager.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>

#include "qrb_robot_base_manager/qrc_utils.hpp"
#include "qrc_msg_management.h"
#include "time_sync_msg.h"

namespace qrb
{
namespace robot_base_manager
{
struct timespec operator+(const timespec & lhs, const timespec & rhs)
{
  timespec result;
  result.tv_sec = lhs.tv_sec + rhs.tv_sec;
  result.tv_nsec = lhs.tv_nsec + rhs.tv_nsec;
  if (result.tv_nsec >= NS_PER_S) {
    ++result.tv_sec;
    result.tv_nsec -= NS_PER_S;
  }
  return result;
}

timespec operator-(const timespec & lhs, const timespec & rhs)
{
  timespec result;
  result.tv_sec = lhs.tv_sec - rhs.tv_sec;
  result.tv_nsec = lhs.tv_nsec - rhs.tv_nsec;
  if (result.tv_nsec < 0) {
    --result.tv_sec;
    result.tv_nsec += NS_PER_S;
  }
  return result;
}

std::ostream & operator<<(std::ostream & os, const timespec & ts)
{
  os << "sec = " << ts.tv_sec << ", nsec = " << ts.tv_nsec;
  return os;
}

TimeManager::TimeManager()
{
  pipe_ = qrc_get_pipe(TIME_SYNC_PIPE);
  if (pipe_ == nullptr) {
    std::cerr << "get qrc pipe: " << TIME_SYNC_PIPE << " failed" << std::endl;
    return;
  }
  qrc_register_message_cb(pipe_, &TimeManager::qrc_message_handle);
}

TimeManager & TimeManager::get_instance()
{
  static TimeManager instance;
  return instance;
}

void TimeManager::qrc_message_handle(qrc_pipe_s *, void * data, std::size_t, bool)
{
  std::unique_lock<std::mutex> lock(get_instance().time_sync_mtx_);
  auto message = static_cast<struct time_sync_msg_s *>(data);
  switch (message->type) {
    case time_sync_msg_type_e::TIME_LOOP: {
      get_instance().time_loop_received_ = true;
      break;
    }
    case time_sync_msg_type_e::GET_TIME:
      std::cout << "TimeManager: QRC received GET_TIME." << std::endl;
      get_instance().get_time_received_ = true;
      get_instance().mcb_time_.tv_sec = message->sec;
      get_instance().mcb_time_.tv_nsec = (long)message->ns;
      break;
    case time_sync_msg_type_e::SET_TIME:
      std::cout << "TimeManager: QRC received SET_TIME." << std::endl;
      get_instance().set_time_received_ = true;
      break;
    default:
      std::cerr << "TimeManager: qrc message not valid, type: " << message->type << std::endl;
      QrcUtils::dump_message(data, sizeof(time_sync_msg_s));
      break;
  }
  lock.unlock();
  get_instance().time_loop_cond_.notify_one();
}

bool TimeManager::test_transport_latency()
{
  int time_loop_max = 10;
  int count = 0;

  sleep(3);
  struct time_sync_msg_s msg = {};
  msg.type = time_sync_msg_type_e::TIME_LOOP;

  qrc_require_pipe(pipe_);

  auto start = get_local_time();
  while (count++ < time_loop_max) {
    qrc_write_fast(pipe_, &msg, sizeof(msg));
    {
      std::unique_lock<std::mutex> lock(time_sync_mtx_);
      time_loop_cond_.wait(lock, [this] { return time_loop_received_; });
      time_loop_received_ = false;
    }
  }
  qrc_release_pipe(pipe_);

  auto end = get_local_time();

  auto diff = end - start;
  auto latency_ns = (diff.tv_sec * NS_PER_S + diff.tv_nsec) / time_loop_max / 2;
  transport_latency_.tv_sec = latency_ns / NS_PER_S;
  transport_latency_.tv_nsec = latency_ns % NS_PER_S;

  std::cout << "TimeManager: test_transport_latency, latency: " << transport_latency_ << std::endl;

  return true;
}

bool TimeManager::need_time_sync()
{
  auto mcb_time = get_mcb_time();
  auto local_time = get_local_time();
  auto diff = local_time - mcb_time - transport_latency_;

  // if diff > 10 ms
  long long diff_ns = (long long)diff.tv_sec * NS_PER_S + diff.tv_nsec;
  std::cout << "TimeManager: check time diff_ns: " << diff_ns << std::endl;

  if (llabs(diff_ns) > time_param_.time_sync_threshold_ms * NS_PER_MS) {
    return true;
  }
  return false;
}

bool TimeManager::trigger_time_sync()
{
  struct time_sync_msg_s msg;
  msg.type = time_sync_msg_type_e::SET_TIME;

  if (time_param_.test_transport_latency_enable) {
    test_transport_latency();
  }

  qrc_require_pipe(pipe_);

  struct timespec ts = get_local_time() + transport_latency_;
  msg.sec = ts.tv_sec;
  msg.ns = ts.tv_nsec;

  qrc_write_fast(pipe_, &msg, sizeof(msg));
  {
    std::unique_lock<std::mutex> lock(time_sync_mtx_);
    time_loop_cond_.wait(lock, [this] { return set_time_received_; });
    set_time_received_ = false;
  }
  qrc_release_pipe(pipe_);

  std::cout << "TimeManager: trigger_time_sync, sec:" << msg.sec << " ns:" << msg.ns << std::endl;

  return true;
}

struct timespec TimeManager::get_mcb_time()
{
  struct time_sync_msg_s msg;
  msg.type = time_sync_msg_type_e::GET_TIME;

  qrc_require_pipe(pipe_);
  qrc_write_fast(pipe_, &msg, sizeof(msg));
  {
    std::unique_lock<std::mutex> lock(time_sync_mtx_);
    time_loop_cond_.wait(lock, [this] { return get_time_received_; });
    get_time_received_ = false;
  }
  qrc_release_pipe(pipe_);

  std::cout << "TimeManager: get_mcb_time, " << mcb_time_ << std::endl;

  return mcb_time_;
}

struct timespec TimeManager::get_local_time()
{
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  return ts;
}

void TimeManager::set_time_param(const TimeParameter & time_param)
{
  time_param_ = time_param;
}

void TimeManager::start_time_sync()
{
  struct sigevent sev;
  struct itimerspec its;

  sev.sigev_notify = SIGEV_THREAD;
  sev.sigev_value.sival_ptr = &timer_id_;
  sev.sigev_notify_function = &TimeManager::time_sync_handler;
  sev.sigev_notify_attributes = nullptr;
  its.it_value.tv_sec = 1;
  its.it_value.tv_nsec = 0;
  its.it_interval.tv_sec = time_param_.time_sync_interval_sec;
  its.it_interval.tv_nsec = 0;

  std::cout << "TimeManager: time test start" << std::endl;
  if (!test_transport_latency()) {
    std::cerr << "TimeManager: test latency failed" << std::endl;
    return;
  }
  std::cout << "TimeManager: time test done" << std::endl;

  if (timer_create(CLOCK_REALTIME, &sev, &timer_id_) == -1) {
    std::cerr << "TimeManager: fail to create timer" << std::endl;
    return;
  }

  if (timer_settime(timer_id_, 0, &its, nullptr) == -1) {
    std::cerr << "TimeManager: fail to start timer" << std::endl;
    return;
  }
}

void TimeManager::stop_time_sync()
{
  std::cout << "TimeManager: stop time sync" << std::endl;
  timer_delete(timer_id_);
}

void TimeManager::sync_time_if_need()
{
  if (!need_time_sync()) {
    return;
  }
  if (!trigger_time_sync()) {
    std::cerr << "TimeManager: sync time failed" << std::endl;
  }
}

void TimeManager::time_sync_handler(union sigval sv)
{
  TimeManager::get_instance().sync_time_if_need();
}

}  // namespace robot_base_manager
}  // namespace qrb
