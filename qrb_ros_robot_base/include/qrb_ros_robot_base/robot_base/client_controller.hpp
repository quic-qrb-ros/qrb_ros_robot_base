// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ROBOT_BASE__ROBOT_BASE__CLIENT_CONTROLLER_HPP_
#define QRB_ROS_ROBOT_BASE__ROBOT_BASE__CLIENT_CONTROLLER_HPP_

#include "qrb_robot_base_manager/client_manager.hpp"
#include "qrb_ros_robot_base_msgs/srv/get_control_mode.hpp"
#include "qrb_ros_robot_base_msgs/srv/set_control_mode.hpp"
#include "rclcpp/rclcpp.hpp"

namespace qrb_ros
{
namespace robot_base
{
class ClientController
{
public:
  explicit ClientController(rclcpp::Node::SharedPtr node);

  void run();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  rclcpp::Service<qrb_ros_robot_base_msgs::srv::GetControlMode>::SharedPtr get_control_mode_server_;
  rclcpp::Service<qrb_ros_robot_base_msgs::srv::SetControlMode>::SharedPtr set_control_mode_server_;

  void get_control_mode_callback(const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<qrb_ros_robot_base_msgs::srv::GetControlMode::Request> request,
      std::shared_ptr<qrb_ros_robot_base_msgs::srv::GetControlMode::Response> response);
  void set_control_mode_callback(const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<qrb_ros_robot_base_msgs::srv::SetControlMode::Request> request,
      std::shared_ptr<qrb_ros_robot_base_msgs::srv::SetControlMode::Response> response);
};

}  // namespace robot_base
}  // namespace qrb_ros

#endif  // QRB_ROS_ROBOT_BASE__ROBOT_BASE__CLIENT_CONTROLLER_HPP_