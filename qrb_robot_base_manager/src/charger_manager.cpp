// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_robot_base_manager/charger_manager.hpp"

#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <utility>

#include "charger_control_msg.h"
#include "qrb_robot_base_manager/qrc_utils.hpp"

namespace qrb
{
namespace robot_base_manager
{

ChargerManager::ChargerManager()
{
  pipe_ = qrc_get_pipe(CHARGER_PIPE);
  if (pipe_ == nullptr) {
    std::cerr << "get qrc pipe: " << CHARGER_PIPE << " failed" << std::endl;
    return;
  }
  auto callback = [](struct qrc_pipe_s *, void * data, std::size_t, bool) {
    qrc_message_handle(data);
  };
  qrc_register_message_cb(pipe_, callback);
  sleep(3);
}

ChargerManager & ChargerManager::get_instance()
{
  static ChargerManager instance;
  return instance;
}

void ChargerManager::qrc_message_handle(void * data)
{
  std::unique_lock<std::mutex> lock(get_instance().msg_mtx_);

  auto msg = static_cast<struct charger_ctl_msg_s *>(data);
  Error error{};

  switch (msg->cmd_type) {
    case charger_ctl_cmd_e::GET_CTL_VOLTAGE:
      std::cout << "ChargerManager: got voltage: " << msg->cmd_data.voltage << std::endl;
      get_instance().current_state_.voltage = msg->cmd_data.voltage;
      if (get_instance().power_state_cb_) {
        get_instance().power_state_cb_(get_instance().current_state_);
      }
      break;
    case charger_ctl_cmd_e::GET_CTL_CURRENT:
      std::cout << "ChargerManager: got current:" << msg->cmd_data.current << std::endl;
      get_instance().current_state_.current = msg->cmd_data.current;
      if (get_instance().power_state_cb_) {
        get_instance().power_state_cb_(get_instance().current_state_);
      }
      break;
    case charger_ctl_cmd_e::GET_CTL_SM_STATE:
      std::cout << "ChargerManager: got state machine state: " << msg->cmd_data.sm_state
                << std::endl;
      get_instance().current_state_.charger_state = ChargerState(msg->cmd_data.sm_state);
      if (get_instance().power_state_cb_) {
        get_instance().power_state_cb_(get_instance().current_state_);
      }
      break;
    case charger_ctl_cmd_e::GET_CTL_PILE_STATE:
      std::cout << "ChargerManager: got pile state: " << msg->cmd_data.pile_stats << std::endl;
      get_instance().current_state_.pile_stats = msg->cmd_data.pile_stats;
      if (get_instance().pile_stats_cb_) {
        get_instance().pile_stats_cb_(msg->cmd_data.pile_stats);
      }
      break;

    case charger_ctl_cmd_e::GET_CTL_EXCEPTION:
      std::cout << "ChargerManager: got error: " << msg->cmd_data.exception_value << std::endl;
      error.set_type(msg->cmd_data.exception_value);
      error.set_message(error.get_error_messages(msg->cmd_data.exception_value));
      if (get_instance().error_cb_) {
        get_instance().error_cb_(error);
      }
      break;
    case charger_ctl_cmd_e::GET_CTL_IS_CHARGING:
      std::cout << "ChargerManager: got is charging: " << msg->cmd_data.is_charging << std::endl;
      if (get_instance().charging_state_cb_) {
        get_instance().charging_state_cb_(msg->cmd_data.is_charging);
      }
      break;
    case charger_ctl_cmd_e::START_CTL_CHARGING:
      get_instance().start_charging_ack_ = true;
      std::cout << "ChargerManager: start charging success" << std::endl;
      break;
    case charger_ctl_cmd_e::STOP_CTL_CHARGING:
      get_instance().stop_charging_ack_ = true;
      std::cout << "ChargerManager: stop charging success " << std::endl;
      break;
    default:
      std::cerr << "ChargerManager: qrc message not valid, type: " << msg->cmd_type << std::endl;
      QrcUtils::dump_message(data, sizeof(charger_ctl_msg_s));
      break;
  }
  lock.unlock();
  get_instance().msg_cond_.notify_all();
}

void ChargerManager::register_power_state_callback(std::function<void(const PowerState &)> cb)
{
  power_state_cb_ = std::move(cb);
}

void ChargerManager::register_pile_state_callback(std::function<void(const uint32_t)> cb)
{
  pile_stats_cb_ = std::move(cb);
}

void ChargerManager::register_charging_state_callback(std::function<void(const uint32_t)> cb)
{
  charging_state_cb_ = std::move(cb);
}

bool ChargerManager::get_power_state()
{
  // TODO(impl) will not receive GET_CTL_ALL_STATE!
  charger_ctl_msg_s msg;
  msg.cmd_type = charger_ctl_cmd_e::GET_CTL_ALL_STATE;
  if (!QrcUtils::send_message(pipe_, &msg, sizeof(msg))) {
    std::cerr << "ChargerManager: get power state failed" << std::endl;
    return false;
  }
  return true;
}

bool ChargerManager::start_charging()
{
  charger_ctl_msg_s msg;
  msg.cmd_type = charger_ctl_cmd_e::START_CTL_CHARGING;
  if (!QrcUtils::send_message(pipe_, &msg, sizeof(msg))) {
    std::cerr << "ChargerManager: start charging failed" << std::endl;
    return false;
  }
  {
    std::unique_lock<std::mutex> lock(msg_mtx_);
    msg_cond_.wait(lock, [this] { return start_charging_ack_; });
    start_charging_ack_ = false;
  }
  return true;
}

bool ChargerManager::stop_charging()
{
  charger_ctl_msg_s msg;
  msg.cmd_type = charger_ctl_cmd_e::STOP_CTL_CHARGING;
  if (!QrcUtils::send_message(pipe_, &msg, sizeof(msg))) {
    std::cerr << "ChargerManager: stop charging failed" << std::endl;
    return false;
  }
  {
    std::unique_lock<std::mutex> lock(msg_mtx_);
    msg_cond_.wait(lock, [this] { return stop_charging_ack_; });
    stop_charging_ack_ = false;
  }
  return true;
}

bool ChargerManager::get_voltage()
{
  charger_ctl_msg_s msg;
  msg.cmd_type = charger_ctl_cmd_e::GET_CTL_VOLTAGE;
  if (!QrcUtils::send_message(pipe_, &msg, sizeof(msg))) {
    std::cerr << "ChargerManager: get voltage failed" << std::endl;
    return false;
  }
  return true;
}

bool ChargerManager::get_current()
{
  charger_ctl_msg_s msg;
  msg.cmd_type = charger_ctl_cmd_e::GET_CTL_CURRENT;
  if (!QrcUtils::send_message(pipe_, &msg, sizeof(msg))) {
    std::cerr << "ChargerManager: get current failed" << std::endl;
    return false;
  }
  return true;
}

bool ChargerManager::get_pile_state()
{
  charger_ctl_msg_s msg;
  msg.cmd_type = charger_ctl_cmd_e::GET_CTL_PILE_STATE;
  if (!QrcUtils::send_message(pipe_, &msg, sizeof(msg))) {
    std::cerr << "ChargerManager: get pile state failed" << std::endl;
    return false;
  }
  return true;
}

bool ChargerManager::get_charger_state()
{
  charger_ctl_msg_s msg;
  msg.cmd_type = charger_ctl_cmd_e::GET_CTL_SM_STATE;
  if (!QrcUtils::send_message(pipe_, &msg, sizeof(msg))) {
    std::cerr << "ChargerManager: get state machine state failed" << std::endl;
    return false;
  }
  return true;
}

void ChargerManager::register_error_callback(std::function<void(const Error &)> cb)
{
  error_cb_ = std::move(cb);
}

}  // namespace robot_base_manager
}  // namespace qrb
