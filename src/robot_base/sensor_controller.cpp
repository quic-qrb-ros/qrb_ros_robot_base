// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_robot_base/robot_base/sensor_controller.hpp"

#include <utility>

namespace qrb_ros
{
namespace robot_base
{
SensorController::SensorController(rclcpp::Node::SharedPtr node) : node_(std::move(node))
{
  imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("mcb_imu", 30);
}

void SensorController::run()
{
  RCLCPP_INFO(node_->get_logger(), "SensorController: start running...");
  qrb::robot_base_manager::SensorManager::get_instance().register_imu_callback(
      std::bind(&SensorController::publish_imu, this, std::placeholders::_1));
}

void SensorController::publish_imu(const qrb::robot_base_manager::IMU & imu)
{
  sensor_msgs::msg::Imu msg;

  msg.header.frame_id = "mcb_imu";
  msg.header.stamp.sec = imu.timestamp.tv_sec;
  msg.header.stamp.nanosec = imu.timestamp.tv_nsec;

  msg.linear_acceleration.x = imu.accel_x;
  msg.linear_acceleration.y = imu.accel_y;
  msg.linear_acceleration.z = imu.accel_z;
  msg.angular_velocity.x = imu.gyro_x;
  msg.angular_velocity.y = imu.gyro_y;
  msg.angular_velocity.z = imu.gyro_z;

  imu_pub_->publish(msg);
}

}  // namespace robot_base
}  // namespace qrb_ros
