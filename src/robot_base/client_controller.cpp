// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_robot_base/robot_base/client_controller.hpp"

#include <bits/stdint-uintn.h>

#include <chrono>
#include <utility>

#include "qrb_robot_base_manager/client_manager.hpp"

using namespace std::placeholders;

namespace qrb_ros
{
namespace robot_base
{
using ClientManager = qrb::robot_base_manager::ClientManager;
using GetControlMode = qrb_ros_robot_base_msgs::srv::GetControlMode;
using SetControlMode = qrb_ros_robot_base_msgs::srv::SetControlMode;

ClientController::ClientController(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {}

void ClientController::run()
{
  RCLCPP_INFO(node_->get_logger(), "ClientController: start running...");

  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;

  get_control_mode_server_ = node_->create_service<GetControlMode>("get_control_mode",
      std::bind(&ClientController::get_control_mode_callback, this, _1, _2, _3),
      rmw_qos_profile_services_default, callback_group_);

  set_control_mode_server_ = node_->create_service<SetControlMode>("set_control_mode",
      std::bind(&ClientController::set_control_mode_callback, this, _1, _2, _3),
      rmw_qos_profile_services_default, callback_group_);

  RCLCPP_INFO(node_->get_logger(), "ClientController: switch to application control mode");
  ClientManager::get_instance().set_client(qrb::robot_base_manager::Client::application);
}

void ClientController::get_control_mode_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<GetControlMode::Request> request,
    std::shared_ptr<GetControlMode::Response> response)
{
  RCLCPP_INFO(node_->get_logger(), "ClientController: get control mode start...");
  auto client = ClientManager::get_instance().get_client();
  response->mode = static_cast<uint8_t>(client);
  RCLCPP_INFO(node_->get_logger(), "ClientController: get control mode success, mode: %d", client);
}

void ClientController::set_control_mode_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<SetControlMode::Request> request,
    std::shared_ptr<SetControlMode::Response> response)
{
  RCLCPP_INFO(node_->get_logger(), "ClientController: set control mode, mode: %d", request->mode);
  auto client = request->mode;
  response->result = ClientManager::get_instance().set_client(
      static_cast<qrb::robot_base_manager::Client>(client));
  RCLCPP_INFO(node_->get_logger(), "ClientController: set control mode %s",
      response->result == true ? "success" : "failed");
}
}  // namespace robot_base
}  // namespace qrb_ros