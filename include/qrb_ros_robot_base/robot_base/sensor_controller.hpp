// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ROBOT_BASE__ROBOT_BASE__SENSOR_CONTROLLER_HPP_
#define QRB_ROS_ROBOT_BASE__ROBOT_BASE__SENSOR_CONTROLLER_HPP_

#include "qrb_robot_base_manager/sensor_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace qrb_ros
{
namespace robot_base
{
class SensorController
{
public:
  explicit SensorController(rclcpp::Node::SharedPtr node);

  void run();

private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  void publish_imu(const qrb::robot_base_manager::IMU & imu);
};

}  // namespace robot_base
}  // namespace qrb_ros

#endif  // QRB_ROS_ROBOT_BASE__ROBOT_BASE__SENSOR_CONTROLLER_HPP_
