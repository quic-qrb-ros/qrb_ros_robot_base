// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_robot_base/robot_base/charger_controller.hpp"

#include <utility>

using namespace std::placeholders;

namespace qrb_ros
{
namespace robot_base
{
using qrb::robot_base_manager::ChargerManager;
using qrb_ros_robot_base_msgs::msg::ChargerCmd;
using qrb_ros_robot_base_msgs::msg::GetBatteryState;
using ChargerState = qrb::robot_base_manager::ChargerState;

ChargerController::ChargerController(rclcpp::Node::SharedPtr node) : node_(std::move(node))
{
  RCLCPP_INFO(node_->get_logger(), "ChargerController: start running...");

  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;

  battery_pub_ = node_->create_publisher<sensor_msgs::msg::BatteryState>("battery", 30);
  get_battery_sub_ = node_->create_subscription<GetBatteryState>("get_battery_state", 30,
      std::bind(&ChargerController::get_battery_state_callback, this, _1), options);
  charge_cmd_sub_ = node_->create_subscription<ChargerCmd>(
      "charger_cmd", 30, std::bind(&ChargerController::charge_cmd_callback, this, _1), options);
}

void ChargerController::run()
{
  ChargerManager::get_instance().register_power_state_callback(
      std::bind(&ChargerController::publish_battery, this, std::placeholders::_1));
}

void ChargerController::publish_battery(const qrb::robot_base_manager::PowerState & state)
{
  sensor_msgs::msg::BatteryState msg{};

  msg.header.frame_id = "battery";
  msg.header.stamp = rclcpp::Clock().now();

  msg.voltage = state.voltage;
  msg.current = state.current;

  std::map<ChargerState, uint8_t> mapping{
    { ChargerState::unknown, sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN },
    { ChargerState::charging, sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING },
    { ChargerState::idle, sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING },
    { ChargerState::charge_done, sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL },
    { ChargerState::searching, 5 },
    { ChargerState::controlling, 6 },
    { ChargerState::force_charging, 7 },
    { ChargerState::error, 8 },
  };
  if (mapping.find(state.charger_state) != mapping.end()) {
    msg.power_supply_status = mapping.at(state.charger_state);
  } else {
    msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  }

  // TODO(impl) other battery information?

  battery_pub_->publish(msg);
}

void ChargerController::get_battery_state_callback(
    const qrb_ros_robot_base_msgs::msg::GetBatteryState::SharedPtr msg)
{
  RCLCPP_INFO(node_->get_logger(), "ChargerController: get battery state");
  ChargerManager::get_instance().get_power_state();
}

void ChargerController::charge_cmd_callback(
    const qrb_ros_robot_base_msgs::msg::ChargerCmd::SharedPtr cmd)
{
  RCLCPP_INFO(node_->get_logger(), "ChargerController: charger cmd: %d start..", cmd->cmd);
  bool result = false;
  if (cmd->cmd == ChargerCmd::START_CHARGING) {
    result = ChargerManager::get_instance().start_charging();
  } else if (cmd->cmd == ChargerCmd::STOP_CHARGING) {
    result = ChargerManager::get_instance().stop_charging();
  }
  RCLCPP_INFO(node_->get_logger(), "ChargerController: charger cmd end, result: %s",
      result ? "success" : "failed");
}

}  // namespace robot_base
}  // namespace qrb_ros
