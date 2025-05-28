// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_robot_base_manager/emergency_manager.hpp"

#include <iostream>
#include <memory>
#include <mutex>
#include <utility>

#include "emergency_msg.h"
#include "qrb_robot_base_manager/qrc_utils.hpp"

namespace qrb
{
namespace robot_base_manager
{
EmergencyManager::EmergencyManager()
{
  pipe_ = qrc_get_pipe(EMERG_PIPE);
  if (pipe_ == nullptr) {
    std::cerr << "get qrc pipe: " << EMERG_PIPE << " failed" << std::endl;
    return;
  }
  auto callback = [](struct qrc_pipe_s *, void * data, std::size_t, bool) {
    qrc_message_handle(data);
  };
  qrc_register_message_cb(pipe_, callback);
  sleep(3);
}

EmergencyManager & EmergencyManager::get_instance()
{
  static EmergencyManager instance;
  return instance;
}

void EmergencyManager::qrc_message_handle(void * data)
{
  std::unique_lock<std::mutex> lock(get_instance().msg_mtx_);

  auto msg = static_cast<struct emerg_msg_s *>(data);
  EmergencyEvent event{};

  switch (msg->msg_type) {
    case emerg_msg_type_e::ENABLEMENT:
      get_instance().enablement_ack_ = true;
      get_instance().set_emergency_result_ = msg->data.value == 1;
      std::cout << "EmergencyManager: got enablement message: " << msg->data.value << std::endl;
      break;
    case emerg_msg_type_e::EVENT:
      std::cout << "EmergencyManager: got emergency event: " << msg->data.event.type
                << ", trigger sensor: " << msg->data.event.trigger_sensor << std::endl;
      event = { EmergencyEventType(msg->data.event.type), msg->data.event.trigger_sensor };
      get_instance().event_cb_(event);
      break;
    default:
      std::cerr << "EmergencyManager: qrc message not valid, type: " << msg->msg_type << std::endl;
      QrcUtils::dump_message(data, sizeof(emerg_msg_s));
      break;
  }
  lock.unlock();
  get_instance().msg_cond_.notify_all();
}

bool EmergencyManager::set_emergency_state(bool enable)
{
  emerg_msg_s msg{};
  msg.msg_type = emerg_msg_type_e::ENABLEMENT;
  msg.data.value = enable ? 1 : 0;
  if (!QrcUtils::send_message(pipe_, &msg, sizeof(msg))) {
    std::cerr << "EmergencyManager: set emegency failed" << std::endl;
    return false;
  }
  {
    std::unique_lock<std::mutex> lock(msg_mtx_);
    msg_cond_.wait(lock, [&] { return enablement_ack_; });
    enablement_ack_ = false;
  }
  return set_emergency_result_;
}

bool EmergencyManager::emergency_avoidance_enable()
{
  return set_emergency_state(true);
}

bool EmergencyManager::emergency_avoidance_disable()
{
  return set_emergency_state(false);
}

void EmergencyManager::register_emergency_event_callback(
    std::function<void(const EmergencyEvent &)> cb)
{
  event_cb_ = std::move(cb);
}

}  // namespace robot_base_manager
}  // namespace qrb
