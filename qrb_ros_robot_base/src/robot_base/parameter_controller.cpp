// Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_robot_base/robot_base/parameter_controller.hpp"

#include <bits/stdint-uintn.h>

#include <algorithm>
#include <array>
#include <iterator>
#include <utility>

#include "qrb_robot_base_manager/parameter_manager.hpp"

namespace qrb_ros
{
namespace robot_base
{
ParameterController::ParameterController(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {}

void ParameterController::load_parameters()
{
  RCLCPP_INFO(node_->get_logger(), "ParameterController: load parameters start...");
  // load car parameters
  robot_base_param_.car_param.model =
      qrb::robot_base_manager::CarModel(node_->declare_parameter<uint8_t>("car_model", 0));
  robot_base_param_.car_param.kinematic = qrb::robot_base_manager::KinematicModel(
      node_->declare_parameter<uint8_t>("car_kinematic_model", 0));
  robot_base_param_.car_param.wheel_perimeter =
      node_->declare_parameter<float>("car_wheel_perimeter", 0.334);
  robot_base_param_.car_param.wheel_space =
      node_->declare_parameter<float>("car_wheel_space", 0.250);

  // load motion parameters
  robot_base_param_.motion_param.max_speed =
      node_->declare_parameter<float>("motion_max_speed", 1.0);
  robot_base_param_.motion_param.max_angle_speed =
      node_->declare_parameter<float>("motion_max_angle_speed", 1.5);
  robot_base_param_.motion_param.max_position_dist =
      node_->declare_parameter<float>("motion_max_position_distance", 0.5);
  robot_base_param_.motion_param.max_position_angle =
      node_->declare_parameter<float>("motion_max_position_angle", 0.5);
  robot_base_param_.motion_param.max_position_line_speed =
      node_->declare_parameter<float>("motion_max_position_line_speed", 60);
  robot_base_param_.motion_param.max_position_angle_speed =
      node_->declare_parameter<float>("motion_max_position_angle_speed", 11);
  robot_base_param_.motion_param.odom_frequency =
      node_->declare_parameter<int>("motion_odom_frequency", 50);

  node_->declare_parameter(
      "motion_pid_speed", rclcpp::ParameterValue(std::vector<float>{ 400, 200, 0 }));
  auto pid = node_->get_parameter("motion_pid_speed").as_double_array();
  std::copy(std::begin(pid), std::end(pid), robot_base_param_.motion_param.pid_speed);

  node_->declare_parameter(
      "motion_pid_position", rclcpp::ParameterValue(std::vector<float>{ 400, 200, 0 }));
  pid = node_->get_parameter("motion_pid_position").as_double_array();
  std::copy(std::begin(pid), std::end(pid), robot_base_param_.motion_param.pid_position);

  // load sensors parameters
  robot_base_param_.sensor_param.imu_enable =
      node_->declare_parameter<bool>("imu_enable", false) ? 1 : 0;
  robot_base_param_.sensor_param.ultra_enable =
      node_->declare_parameter<bool>("ultra_enable", true) ? 1 : 0;

  robot_base_param_.sensor_param.ultra_quantity =
      node_->declare_parameter<uint8_t>("ultra_quantity", 5);

  // load remote controller parameters
  robot_base_param_.rc_param.rc_enable = node_->declare_parameter<bool>("rc_enable", true) ? 1 : 0;
  robot_base_param_.rc_param.max_speed = node_->declare_parameter<float>("rc_max_speed", 0.8);
  robot_base_param_.rc_param.max_angle_speed =
      node_->declare_parameter<float>("rc_max_angle_speed", 2.0);

  // load scale parameters
  node_->declare_parameter("scale_speed", rclcpp::ParameterValue(std::vector<float>{ 1, 1 }));
  node_->declare_parameter("scale_position", rclcpp::ParameterValue(std::vector<float>{ 1, 1 }));
  node_->declare_parameter("scale_speed_odom", rclcpp::ParameterValue(std::vector<float>{ 1, 1 }));
  node_->declare_parameter(
      "scale_position_odom", rclcpp::ParameterValue(std::vector<float>{ 1, 1 }));
  auto scale_speed = node_->get_parameter("scale_speed").as_double_array();
  auto scale_position = node_->get_parameter("scale_position").as_double_array();
  auto scale_speed_odom = node_->get_parameter("scale_speed_odom").as_double_array();
  auto scale_position_odom = node_->get_parameter("scale_position_odom").as_double_array();

  std::copy(std::begin(scale_speed), std::end(scale_speed), robot_base_param_.scale_param.speed);
  std::copy(
      std::begin(scale_position), std::end(scale_position), robot_base_param_.scale_param.position);
  std::copy(std::begin(scale_speed_odom), std::end(scale_speed_odom),
      robot_base_param_.scale_param.speed_odom);
  std::copy(std::begin(scale_position_odom), std::end(scale_position_odom),
      robot_base_param_.scale_param.position_odom);

  // load obstacle avoidance parameters
  robot_base_param_.oba_param.bottom_dist =
      node_->declare_parameter<float>("oba_bottom_distance", 0.05);
  robot_base_param_.oba_param.side_dist =
      node_->declare_parameter<float>("oba_side_distance", 0.15);
  robot_base_param_.oba_param.front_dist =
      node_->declare_parameter<float>("oba_front_distance", 0.15);

  robot_base_param_.time_param.test_transport_latency_enable =
      node_->declare_parameter<bool>("test_transport_latency_enable", false) ? 1 : 0;
  robot_base_param_.time_param.time_sync_interval_sec =
      node_->declare_parameter<int>("time_sync_interval_sec", 300);
  robot_base_param_.time_param.time_sync_threshold_ms =
      node_->declare_parameter<int>("time_sync_threshold_ms", 10);

  RCLCPP_INFO(node_->get_logger(), "ParameterController: load parameters end");
}

qrb::robot_base_manager::RobotBaseParameter ParameterController::get_parameters()
{
  return robot_base_param_;
}

}  // namespace robot_base
}  // namespace qrb_ros