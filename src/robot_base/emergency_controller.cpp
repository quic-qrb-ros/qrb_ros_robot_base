// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_robot_base/robot_base/emergency_controller.hpp"

#include <bits/stdint-uintn.h>

#include <chrono>
#include <utility>

using namespace std::placeholders;

namespace qrb_ros
{
namespace robot_base
{
using EmergencyManager = qrb::robot_base_manager::EmergencyManager;
using EmergencyCmd = qrb_ros_robot_base_msgs::srv::EmergencyCmd;
using Exception = qrb_ros_robot_base_msgs::msg::Exception;

EmergencyController::EmergencyController(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {}

void EmergencyController::run()
{
  bool ultra_enable = true;

  RCLCPP_INFO(node_->get_logger(), "EmergencyController: start running...");
  node_->get_parameter("ultra_enable", ultra_enable);
  if (ultra_enable == false) {
    RCLCPP_ERROR(node_->get_logger(),
        "EmergencyController: ultra has been disable, will not create emergency service...");
    return;
  }

  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;

  emergency_cmd_server_ = node_->create_service<EmergencyCmd>("emergency_cmd",
      std::bind(&EmergencyController::emergency_avoidance_cmd_callback, this, _1, _2, _3),
      rmw_qos_profile_services_default, callback_group_);

  emergency_event_pub_ = node_->create_publisher<Exception>("robot_base_exception", 10);

  EmergencyManager::get_instance().register_emergency_event_callback(
      std::bind(&EmergencyController::publish_emergency_event, this, _1));
}

void EmergencyController::emergency_avoidance_cmd_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<EmergencyCmd::Request> request,
    std::shared_ptr<EmergencyCmd::Response> response)
{
  RCLCPP_INFO(
      node_->get_logger(), "EmergencyController: set emergency avoidance, mode: %d", request->mode);
  auto result = false;
  if (true == request->mode) {
    result = EmergencyManager::get_instance().emergency_avoidance_enable();
  } else {
    result = EmergencyManager::get_instance().emergency_avoidance_disable();
  }
  response->result = result;
  RCLCPP_INFO(node_->get_logger(), "EmergencyController: set emergency result: %d", result);
}

void EmergencyController::publish_emergency_event(
    const qrb::robot_base_manager::EmergencyEvent & event)
{
  Exception msg;
  msg.type = Exception::EMERGENCY;
  msg.event = static_cast<uint8_t>(event.type);
  msg.trigger_sensor = event.trigger_sensor;
  emergency_event_pub_->publish(msg);
}

}  // namespace robot_base
}  // namespace qrb_ros
