// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_robot_base/robot_base/error_controller.hpp"

#include <bits/stdint-uintn.h>

#include <chrono>
#include <utility>

#include "qrb_robot_base_manager/charger_manager.hpp"
#include "qrb_robot_base_manager/motion_manager.hpp"
#include "qrb_robot_base_manager/watchdog.hpp"

using namespace std::placeholders;

namespace qrb_ros
{
namespace robot_base
{
using qrb::robot_base_manager::ChargerManager;
using qrb::robot_base_manager::MotionManager;
using qrb::robot_base_manager::Watchdog;
using qrb_ros_robot_base_msgs::msg::Error;

ErrorController::ErrorController(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {}

void ErrorController::run()
{
  RCLCPP_INFO(node_->get_logger(), "ErrorController: start running...");

  error_pub_ = node_->create_publisher<Error>("robot_base_error", 30);

  MotionManager::get_instance().register_error_callback(
      [this](const qrb::robot_base_manager::Error & e) { publish_error(e); });

  ChargerManager::get_instance().register_error_callback(
      [this](const qrb::robot_base_manager::Error & e) { publish_error(e); });

  Watchdog::get_instance().register_error_callback(
      [this](const qrb::robot_base_manager::Error & e) { publish_error(e); });
}

void ErrorController::publish_error(const qrb::robot_base_manager::Error & error)
{
  Error msg;
  msg.type = error.type();
  msg.message = error.message();
  error_pub_->publish(msg);
}

}  // namespace robot_base
}  // namespace qrb_ros