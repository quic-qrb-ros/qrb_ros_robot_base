// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_robot_base_manager/motion_manager.hpp"

#include <iostream>
#include <memory>
#include <mutex>
#include <utility>

#include "motion_msg.h"
#include "qrb_robot_base_manager/qrc_utils.hpp"

namespace qrb
{
namespace robot_base_manager
{
MotionManager::MotionManager()
{
  pipe_ = qrc_get_pipe(MOTION_PIPE);
  if (pipe_ == nullptr) {
    std::cerr << "get qrc pipe: " << MOTION_PIPE << " failed" << std::endl;
    return;
  }
  auto callback = [](struct qrc_pipe_s *, void * data, std::size_t, bool) {
    qrc_message_handle(data);
  };
  qrc_register_message_cb(pipe_, callback);
}

MotionManager & MotionManager::get_instance()
{
  static MotionManager instance;
  return instance;
}

void MotionManager::qrc_message_handle(void * data)
{
  std::unique_lock<std::mutex> lock(get_instance().msg_mtx_);

  auto msg = static_cast<struct motion_control_msg_s *>(data);
  Error error{};

  switch (msg->msg_type) {
    case control_msg_type_e::SWITCH_MODE:
      get_instance().set_control_mode_ack_ = true;
      get_instance().control_mode_ = static_cast<MotionMode>(msg->data.mode);
      std::cout << "MotionManager: got switch mode message, mode: " << msg->data.mode << std::endl;
      break;
    case control_msg_type_e::SET_SPEED:
      std::cout << "MotionManager: set speed success" << std::endl;
      break;
    case control_msg_type_e::SET_POSITION:
      get_instance().set_position_ack_ = true;
      std::cout << "MotionManager: set position success" << std::endl;
      break;
#ifdef ENABLE_TEST
    case control_msg_type_e::SET_EMERGENCY:
      get_instance().set_emergency_ack_ = true;
      get_instance().set_emergency_result_ = msg->data.emergency == 1;
      std::cout << "MotionManager: set emergency result: " << msg->data.emergency << std::endl;
      break;
#endif
    case control_msg_type_e::MOTOR_DRIVER_STATUS:
      std::cout << "MotionManager: motor driver error: " << msg->data.motor_driver_status
                << std::endl;
      error.set_type(msg->data.motor_driver_status);
      error.set_message(error.get_error_messages(msg->data.motor_driver_status));
      if (get_instance().error_cb_) {
        get_instance().error_cb_(error);
      }
      break;
    default:
      std::cerr << "MotionManager: qrc message not valid, type: " << msg->msg_type << std::endl;
      QrcUtils::dump_message(data, sizeof(motion_control_msg_s));
      break;
  }
  lock.unlock();
  get_instance().msg_cond_.notify_all();
}

void MotionManager::set_speed(float linear, float angular)
{
  motion_control_msg_s msg{};
  msg.msg_type = control_msg_type_e::SET_SPEED;
  msg.data.speed_cmd = { linear, angular };
  if (!QrcUtils::send_message(pipe_, &msg, sizeof(msg))) {
    std::cerr << "MotionManager: set speed failed" << std::endl;
  }
}

bool MotionManager::forward(float distance)
{
  motion_control_msg_s msg{};
  msg.msg_type = control_msg_type_e::SET_POSITION;
  // TODO pose_type?
  msg.data.position_cmd = { true, distance };
  if (!QrcUtils::send_message(pipe_, &msg, sizeof(msg))) {
    std::cerr << "MotionManager: forward failed" << std::endl;
    return false;
  }
  {
    std::unique_lock<std::mutex> lock(msg_mtx_);
    msg_cond_.wait(lock, [&] { return set_position_ack_; });
    set_position_ack_ = false;
  }
  // TODO: check result
  return true;
}

bool MotionManager::rotate(float angle)
{
  motion_control_msg_s msg{};
  msg.msg_type = control_msg_type_e::SET_POSITION;
  // TODO pose_type?
  msg.data.position_cmd = { false, angle };
  if (!QrcUtils::send_message(pipe_, &msg, sizeof(msg))) {
    std::cerr << "MotionManager: rotate failed" << std::endl;
    return false;
  }
  {
    std::unique_lock<std::mutex> lock(msg_mtx_);
    msg_cond_.wait(lock, [&] { return set_position_ack_; });
    set_position_ack_ = false;
  }
  // TODO: check result
  return true;
}

void MotionManager::stop()
{
  motion_control_msg_s msg{};
  msg.msg_type = control_msg_type_e::SET_SPEED;
  msg.data.speed_cmd = { 0, 0 };
  if (!QrcUtils::send_message(pipe_, &msg, sizeof(msg))) {
    std::cerr << "MotionManager: set speed failed" << std::endl;
  }
}

#ifdef ENABLE_TEST
bool MotionManager::set_emergency(bool enable)
{
  motion_control_msg_s msg{};
  msg.msg_type = control_msg_type_e::SET_EMERGENCY;
  msg.data.emergency = enable ? 1 : 0;
  if (!QrcUtils::send_message(pipe_, &msg, sizeof(msg))) {
    std::cerr << "MotionManager: stop failed" << std::endl;
    return false;
  }
  {
    std::unique_lock<std::mutex> lock(msg_mtx_);
    msg_cond_.wait(lock, [&] { return set_emergency_ack_; });
    set_emergency_ack_ = false;
  }
  return set_emergency_result_;
}
#endif

bool MotionManager::set_motion_mode(const MotionMode & mode)
{
  motion_control_msg_s msg{};
  msg.msg_type = control_msg_type_e::SWITCH_MODE;
  msg.data.mode = control_mode_e(mode);
  if (!QrcUtils::send_message(pipe_, &msg, sizeof(msg))) {
    std::cerr << "MotionManager: set motion mode failed" << std::endl;
    return false;
  }
  {
    std::unique_lock<std::mutex> lock(msg_mtx_);
    msg_cond_.wait(lock, [&] { return set_control_mode_ack_; });
    set_control_mode_ack_ = false;
  }
  if (control_mode_ != mode) {
    return false;
  }
  return true;
}

void MotionManager::register_error_callback(std::function<void(const Error &)> cb)
{
  error_cb_ = std::move(cb);
}

}  // namespace robot_base_manager
}  // namespace qrb
