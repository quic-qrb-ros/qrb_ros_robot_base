// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ROBOT_BASE__ROBOT_BASE__ERROR_CONTROLLER_HPP_
#define QRB_ROS_ROBOT_BASE__ROBOT_BASE__ERROR_CONTROLLER_HPP_

#include "qrb_robot_base_manager/error.hpp"
#include "qrb_ros_robot_base_msgs/msg/error.hpp"
#include "rclcpp/rclcpp.hpp"

namespace qrb_ros
{
namespace robot_base
{
class ErrorController
{
public:
  explicit ErrorController(rclcpp::Node::SharedPtr node);
  void run();

private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<qrb_ros_robot_base_msgs::msg::Error>::SharedPtr error_pub_;
  void publish_error(const qrb::robot_base_manager::Error & error);
};

}  // namespace robot_base
}  // namespace qrb_ros

#endif  // QRB_ROS_ROBOT_BASE__ROBOT_BASE__ERROR_CONTROLLER_HPP_
