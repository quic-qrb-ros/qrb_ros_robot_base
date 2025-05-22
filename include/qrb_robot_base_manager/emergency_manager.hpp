// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROBOT_BASE_MANAGER__EMERGENCY_MANAGER_HPP_
#define QRB_ROBOT_BASE_MANAGER__EMERGENCY_MANAGER_HPP_

#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>

#include "qrc_msg_management.h"

namespace qrb
{
namespace robot_base_manager
{
enum class EmergencyEventType
{
  enter,
  exit,
};

struct EmergencyEvent
{
  EmergencyEventType type;
  int trigger_sensor;
};

class EmergencyManager
{
private:
  EmergencyManager();

public:
  EmergencyManager(const EmergencyManager &) = delete;
  EmergencyManager & operator=(const EmergencyManager &) = delete;
  EmergencyManager(const EmergencyManager &&) = delete;
  EmergencyManager & operator=(const EmergencyManager &&) = delete;

  static EmergencyManager & get_instance();
  static void qrc_message_handle(void * data);

  bool emergency_avoidance_enable();
  bool emergency_avoidance_disable();
  void register_emergency_event_callback(std::function<void(const EmergencyEvent &)>);

private:
  bool set_emergency_state(bool enable);
  struct qrc_pipe_s * pipe_ = nullptr;
  std::mutex msg_mtx_;
  std::condition_variable msg_cond_;
  bool enablement_ack_ = false;

  bool set_emergency_result_ = false;

  std::function<void(const EmergencyEvent &)> event_cb_;
};

}  // namespace robot_base_manager
}  // namespace qrb

#endif  // QRB_ROBOT_BASE_MANAGER__EMERGENCY_MANAGER_HPP_
