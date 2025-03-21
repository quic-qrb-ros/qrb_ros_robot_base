// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROBOT_BASE_MANAGER__CHARGER_MANAGER_HPP_
#define QRB_ROBOT_BASE_MANAGER__CHARGER_MANAGER_HPP_

#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>

#include "qrb_robot_base_manager/error.hpp"
#include "qrc_msg_management.h"

namespace qrb
{
namespace robot_base_manager
{

enum class ChargerState
{
  unknown,
  idle = 1,
  searching,
  controlling,
  force_charging,
  charging,
  charge_done,
  error
};

struct PowerState
{
  ChargerState charger_state;
  float voltage;
  float current;
  uint32_t pile_stats;
  uint32_t is_charging;
};

class ChargerManager
{
private:
  ChargerManager();

public:
  ChargerManager(const ChargerManager &) = delete;
  ChargerManager & operator=(const ChargerManager &) = delete;
  ChargerManager(const ChargerManager &&) = delete;
  ChargerManager & operator=(const ChargerManager &&) = delete;

  static ChargerManager & get_instance();
  static void qrc_message_handle(void * data);

  void register_power_state_callback(std::function<void(const PowerState &)> cb);
  bool start_charging();
  bool stop_charging();

  bool get_voltage();
  bool get_current();
  bool get_pile_state();
  bool get_charger_state();
  PowerState get_power_state();

  void register_error_callback(std::function<void(const Error &)>);
  void register_pile_state_callback(std::function<void(const uint32_t)> cb);
  void register_charging_state_callback(std::function<void(const uint32_t)> cb);

private:
  struct qrc_pipe_s * pipe_ = nullptr;
  std::mutex msg_mtx_;
  std::condition_variable msg_cond_;
  bool start_charging_ack_ = false;
  bool stop_charging_ack_ = false;
  bool is_updating_ = false;
  PowerState current_state_{};
  bool get_voltage_ack_ = false;
  bool get_current_ack_ = false;
  bool get_state_machine_ack_ = false;

  std::function<void(const PowerState &)> power_state_cb_{ nullptr };
  std::function<void(const Error &)> error_cb_{ nullptr };
  std::function<void(const uint32_t)> pile_stats_cb_{ nullptr };
  std::function<void(const uint32_t)> charging_state_cb_{ nullptr };
};
}  // namespace robot_base_manager
}  // namespace qrb

#endif  // QRB_ROBOT_BASE_MANAGER__CHARGER_MANAGER_HPP_
