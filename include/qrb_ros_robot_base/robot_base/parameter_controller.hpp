// Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ROBOT_BASE__ROBOT_BASE__PARAMETER_CONTROLLER_HPP_
#define QRB_ROS_ROBOT_BASE__ROBOT_BASE__PARAMETER_CONTROLLER_HPP_

#include "qrb_robot_base_manager/parameter_manager.hpp"
#include "rclcpp/rclcpp.hpp"

namespace qrb_ros
{
namespace robot_base
{

class ParameterController
{
public:
  explicit ParameterController(rclcpp::Node::SharedPtr node);
  void load_parameters();
  qrb::robot_base_manager::RobotBaseParameter get_parameters();

private:
  rclcpp::Node::SharedPtr node_;
  qrb::robot_base_manager::RobotBaseParameter robot_base_param_{};
};

}  // namespace robot_base
}  // namespace qrb_ros

#endif  // QRB_ROS_ROBOT_BASE__ROBOT_BASE__PARAMETER_CONTROLLER_HPP_