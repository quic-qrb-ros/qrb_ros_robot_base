// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ROBOT_BASE__ROBOT_BASE__MOTION_CONTROLLER_HPP_
#define QRB_ROS_ROBOT_BASE__ROBOT_BASE__MOTION_CONTROLLER_HPP_

#include "nav_msgs/msg/odometry.hpp"
#include "qrb_ros_robot_base_msgs/msg/stop.hpp"
#include "qrb_ros_robot_base_msgs/srv/forward.hpp"
#include "qrb_ros_robot_base_msgs/srv/rotate.hpp"
#include "qrb_ros_robot_base_msgs/srv/set_motion_mode.hpp"
#include "rclcpp/rclcpp.hpp"

namespace qrb_ros
{
namespace robot_base
{
class MotionController
{
public:
  explicit MotionController(rclcpp::Node::SharedPtr node);

  void run();
  void stop();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  rclcpp::Service<qrb_ros_robot_base_msgs::srv::SetMotionMode>::SharedPtr set_mode_server_;
  rclcpp::Service<qrb_ros_robot_base_msgs::srv::Forward>::SharedPtr forward_server_;
  rclcpp::Service<qrb_ros_robot_base_msgs::srv::Rotate>::SharedPtr rotate_server_;

  rclcpp::Subscription<qrb_ros_robot_base_msgs::msg::Stop>::SharedPtr stop_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr speed_sub_;

  void set_mode_callback(const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<qrb_ros_robot_base_msgs::srv::SetMotionMode::Request> request,
      std::shared_ptr<qrb_ros_robot_base_msgs::srv::SetMotionMode::Response> response);

  void speed_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr speed);
  void forward_callback(const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<qrb_ros_robot_base_msgs::srv::Forward::Request> request,
      std::shared_ptr<qrb_ros_robot_base_msgs::srv::Forward::Response> response);
  void rotate_callback(const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<qrb_ros_robot_base_msgs::srv::Rotate::Request> request,
      std::shared_ptr<qrb_ros_robot_base_msgs::srv::Rotate::Response> response);
  void stop_callback(const qrb_ros_robot_base_msgs::msg::Stop::SharedPtr stop);
};

}  // namespace robot_base
}  // namespace qrb_ros

#endif  // QRB_ROS_ROBOT_BASE__ROBOT_BASE__MOTION_CONTROLLER_HPP_
