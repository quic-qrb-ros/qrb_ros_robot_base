// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ROBOT_BASE__ROBOT_BASE__CHARGER_CONTROLLER_HPP_
#define QRB_ROS_ROBOT_BASE__ROBOT_BASE__CHARGER_CONTROLLER_HPP_

#include "qrb_robot_base_manager/charger_manager.hpp"
#include "qrb_ros_robot_base_msgs/msg/charger_cmd.hpp"
#include "qrb_ros_robot_base_msgs/srv/get_battery_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

namespace qrb_ros
{
namespace robot_base
{
class ChargerController
{
public:
  explicit ChargerController(rclcpp::Node::SharedPtr node);
  void run();

private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;

  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Subscription<qrb_ros_robot_base_msgs::msg::ChargerCmd>::SharedPtr charge_cmd_sub_;
  rclcpp::Service<qrb_ros_robot_base_msgs::srv::GetBatteryState>::SharedPtr get_battery_state_server_;

  void publish_battery(const qrb::robot_base_manager::PowerState & state);
  void get_battery_state_callback(const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<qrb_ros_robot_base_msgs::srv::GetBatteryState::Request> request,
      std::shared_ptr<qrb_ros_robot_base_msgs::srv::GetBatteryState::Response> response);

  void charge_cmd_callback(const qrb_ros_robot_base_msgs::msg::ChargerCmd::SharedPtr cmd);
};

}  // namespace robot_base
}  // namespace qrb_ros

#endif  // QRB_ROS_ROBOT_BASE__ROBOT_BASE__CHARGER_CONTROLLER_HPP_
