// Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ROBOT_BASE__ROBOT_BASE_CONTROL_HPP_
#define QRB_ROS_ROBOT_BASE__ROBOT_BASE_CONTROL_HPP_

#include "qrb_ros_robot_base/robot_base/charger_controller.hpp"
#include "qrb_ros_robot_base/robot_base/client_controller.hpp"
#include "qrb_ros_robot_base/robot_base/emergency_controller.hpp"
#include "qrb_ros_robot_base/robot_base/error_controller.hpp"
#include "qrb_ros_robot_base/robot_base/motion_controller.hpp"
#include "qrb_ros_robot_base/robot_base/odom_controller.hpp"
#include "qrb_ros_robot_base/robot_base/parameter_controller.hpp"
#include "qrb_ros_robot_base/robot_base/sensor_controller.hpp"

namespace qrb_ros
{
namespace robot_base
{
class RobotBase : public rclcpp::Node
{
public:
  explicit RobotBase(const rclcpp::NodeOptions & options);
  void run();
  void shutdown();

private:
  std::shared_ptr<ParameterController> parameter_ctl_;
  std::shared_ptr<ClientController> client_ctl_;
  std::shared_ptr<MotionController> motion_ctl_;
  std::shared_ptr<OdomController> odom_ctl_;
  std::shared_ptr<SensorController> sensor_ctl_;
  std::shared_ptr<ChargerController> charger_ctl_;
  std::shared_ptr<EmergencyController> emergency_ctl_;
  std::shared_ptr<ErrorController> error_ctl_;
};

}  // namespace robot_base
}  // namespace qrb_ros

#endif  // QRB_ROS_ROBOT_BASE__ROBOT_BASE_CONTROL_HPP_
