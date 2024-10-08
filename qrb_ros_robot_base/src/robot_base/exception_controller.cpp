// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_robot_base/robot_base/exception_controller.hpp"

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
using qrb_ros_robot_base_msgs::msg::Exception;

ExceptionController::ExceptionController(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {}

void ExceptionController::run()
{
  RCLCPP_INFO(node_->get_logger(), "ExceptionController: start running...");

  exception_pub_ = node_->create_publisher<Exception>("exception", 30);

  MotionManager::get_instance().register_exception_callback(
      [this](const qrb::robot_base_manager::MotionException & e) { publish_exception(e); });

  ChargerManager::get_instance().register_exception_callback(
      [this](const qrb::robot_base_manager::ChargerException & e) { publish_exception(e); });

  Watchdog::get_instance().register_exception_callback(
      [this](const qrb::robot_base_manager::WatchdogException & e) { publish_exception(e); });
}

void ExceptionController::publish_exception(const qrb::robot_base_manager::Exception & exception)
{
  Exception msg;
  msg.type = exception.type();
  msg.id = exception.id();
  msg.message = exception.message();
  exception_pub_->publish(msg);
}

}  // namespace robot_base
}  // namespace qrb_ros