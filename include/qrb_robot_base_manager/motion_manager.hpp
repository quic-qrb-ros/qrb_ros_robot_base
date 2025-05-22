// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROBOT_BASE_MANAGER__MOTION_MANAGER_HPP_
#define QRB_ROBOT_BASE_MANAGER__MOTION_MANAGER_HPP_

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
enum class MotionMode
{
  inactive,
  speed,
  position,
  torque,
#ifdef ENABLE_TEST
  driver_error,
#endif
};

class MotionManager
{
private:
  MotionManager();

public:
  MotionManager(const MotionManager &) = delete;
  MotionManager & operator=(const MotionManager &) = delete;
  MotionManager(const MotionManager &&) = delete;
  MotionManager & operator=(const MotionManager &&) = delete;

  static MotionManager & get_instance();
  static void qrc_message_handle(void * data);

  /// set robot base speed, must in spped control mode
  ///
  /// @param linear linear velocity
  /// @param angular angular velocity
  ///
  void set_speed(float linear, float angular);

  /// forward robot base
  ///
  /// @param distance it means go back if negative
  ///
  bool forward(float distance);

  /// rotate robot base
  bool rotate(float angle);

  /// quick stop
  void stop();

#ifdef ENABLE_TEST
  bool set_emergency(bool enable);
#endif

  bool set_motion_mode(const MotionMode & mode);

  void register_error_callback(std::function<void(const Error &)>);

private:
  struct qrc_pipe_s * pipe_ = nullptr;
  std::mutex msg_mtx_;
  std::condition_variable msg_cond_;
  bool set_position_ack_ = false;
  bool set_control_mode_ack_ = false;
#ifdef ENABLE_TEST
  bool set_emergency_ack_ = false;
  bool set_emergency_result_ = false;
#endif
  MotionMode control_mode_{ MotionMode::speed };

  std::function<void(const Error &)> error_cb_;
};

}  // namespace robot_base_manager
}  // namespace qrb

#endif  // QRB_ROBOT_BASE_MANAGER__MOTION_MANAGER_HPP_