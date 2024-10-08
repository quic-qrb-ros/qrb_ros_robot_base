// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ROBOT_BASE__ROBOT_BASE__EXCEPTION_CONTROLLER_HPP_
#define QRB_ROS_ROBOT_BASE__ROBOT_BASE__EXCEPTION_CONTROLLER_HPP_

#include "qrb_robot_base_manager/exception.hpp"
#include "qrb_ros_robot_base_msgs/msg/exception.hpp"
#include "rclcpp/rclcpp.hpp"

namespace qrb_ros
{
namespace robot_base
{
class ExceptionController
{
public:
  explicit ExceptionController(rclcpp::Node::SharedPtr node);
  void run();

private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<qrb_ros_robot_base_msgs::msg::Exception>::SharedPtr exception_pub_;
  void publish_exception(const qrb::robot_base_manager::Exception & exception);
};

}  // namespace robot_base
}  // namespace qrb_ros

#endif  // QRB_ROS_ROBOT_BASE__ROBOT_BASE__EXCEPTION_CONTROLLER_HPP_
