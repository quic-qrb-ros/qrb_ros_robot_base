// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_robot_base/robot_base/motion_controller.hpp"

#include <bits/stdint-uintn.h>
#include <rmw/qos_profiles.h>

#include <algorithm>
#include <array>
#include <functional>
#include <iterator>
#include <utility>

#include "qrb_robot_base_manager/motion_manager.hpp"

using namespace std::placeholders;

namespace qrb_ros
{
namespace robot_base
{
using MotionManager = qrb::robot_base_manager::MotionManager;
using MotionMode = qrb::robot_base_manager::MotionMode;
using SetMotionMode = qrb_ros_robot_base_msgs::srv::SetMotionMode;
using Forward = qrb_ros_robot_base_msgs::srv::Forward;
using Rotate = qrb_ros_robot_base_msgs::srv::Rotate;
using Stop = qrb_ros_robot_base_msgs::msg::Stop;

MotionController::MotionController(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {}

void MotionController::run()
{
  RCLCPP_INFO(node_->get_logger(), "MotionController: start running...");

  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;

  set_mode_server_ = node_->create_service<SetMotionMode>("set_motion_mode",
      std::bind(&MotionController::set_mode_callback, this, _1, _2, _3),
      rmw_qos_profile_services_default, callback_group_);

  speed_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 100, std::bind(&MotionController::speed_cmd_callback, this, _1), options);

  stop_sub_ = node_->create_subscription<Stop>(
      "stop", 100, std::bind(&MotionController::stop_callback, this, _1), options);

  RCLCPP_INFO(node_->get_logger(), "MotionController: auto set speed motion mode");
  MotionManager::get_instance().set_motion_mode(MotionMode::speed);
}

void MotionController::stop()
{
  RCLCPP_INFO(node_->get_logger(), "MotionController: stop");
  MotionManager::get_instance().stop();
}

void MotionController::set_mode_callback(const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<SetMotionMode::Request> request,
    std::shared_ptr<SetMotionMode::Response> response)
{
  RCLCPP_INFO(node_->get_logger(), "MotionController: set motion mode, mode: %d", request->mode);

  // for test only: motion emergency brake, 5:enable 6:disable
  if (request->mode == 5 || request->mode == 6) {
#ifdef ENABLE_TEST
    response->result = MotionManager::get_instance().set_emergency(request->mode == 5);
#endif
    return;
  }

  auto ret = MotionManager::get_instance().set_motion_mode(MotionMode(request->mode));
  response->result = ret;
  RCLCPP_INFO(node_->get_logger(), "MotionController: set motion mode %s",
      ret == true ? "success" : "failed");
}

void MotionController::speed_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr speed)
{
  // TODO(impl) speed x,y,z => x
  MotionManager::get_instance().set_speed(speed->linear.x, speed->angular.z);
}

void MotionController::forward_callback(const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<Forward::Request> request,
    std::shared_ptr<Forward::Response> response)
{
  RCLCPP_WARN(node_->get_logger(), "MotionController: forward not implementation");
}

void MotionController::rotate_callback(const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<Rotate::Request> request,
    std::shared_ptr<Rotate::Response> response)
{
  RCLCPP_WARN(node_->get_logger(), "MotionController: rotate not implementation");
}

void MotionController::stop_callback(const Stop::SharedPtr stop)
{
  RCLCPP_INFO(node_->get_logger(), "MotionController: stop");
  MotionManager::get_instance().stop();
}

}  // namespace robot_base
}  // namespace qrb_ros
