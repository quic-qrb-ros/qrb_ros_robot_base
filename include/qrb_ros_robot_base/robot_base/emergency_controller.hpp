// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ROBOT_BASE__ROBOT_BASE__EMERGENCY_CONTROLLER_HPP_
#define QRB_ROS_ROBOT_BASE__ROBOT_BASE__EMERGENCY_CONTROLLER_HPP_

#include "qrb_robot_base_manager/emergency_manager.hpp"
#include "qrb_ros_robot_base_msgs/msg/exception.hpp"
#include "qrb_ros_robot_base_msgs/srv/emergency_cmd.hpp"
#include "rclcpp/rclcpp.hpp"

namespace qrb_ros
{
namespace robot_base
{
class EmergencyController
{
public:
  explicit EmergencyController(rclcpp::Node::SharedPtr node);

  void run();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  rclcpp::Publisher<qrb_ros_robot_base_msgs::msg::Exception>::SharedPtr emergency_event_pub_;
  rclcpp::Service<qrb_ros_robot_base_msgs::srv::EmergencyCmd>::SharedPtr emergency_cmd_server_;

  void publish_emergency_event(const qrb::robot_base_manager::EmergencyEvent & event);
  void emergency_avoidance_cmd_callback(const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<qrb_ros_robot_base_msgs::srv::EmergencyCmd::Request> request,
      std::shared_ptr<qrb_ros_robot_base_msgs::srv::EmergencyCmd::Response> response);
};

}  // namespace robot_base
}  // namespace qrb_ros

#endif  // QRB_ROS_ROBOT_BASE__ROBOT_BASE__EMERGENCY_CONTROLLER_HPP_
