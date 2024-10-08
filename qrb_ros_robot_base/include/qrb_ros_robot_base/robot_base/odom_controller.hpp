// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ROBOT_BASE__ROBOT_BASE__ODOM_CONTROLLER_HPP_
#define QRB_ROS_ROBOT_BASE__ROBOT_BASE__ODOM_CONTROLLER_HPP_

#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace qrb_ros
{
namespace robot_base
{
class OdomController
{
public:
  explicit OdomController(rclcpp::Node::SharedPtr node);

  void run();

private:
  void publish_odom_velocity(float vx, float vz, timespec ts);
  void publish_odom_position(float px, float pz, timespec ts);
  void publish_tf(const geometry_msgs::msg::Transform & tf, timespec ts);
  bool compare_timespec(const timespec & ts1, const timespec & ts2);
  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  bool tf_enable_ = true;

  timespec last_ts_{};
  geometry_msgs::msg::Pose last_pos_{};
  double last_angle_ = 0;
  double last_odom_vx_ = 0;
  double last_odom_vy_ = 0;
  double last_vz_ = 0;
  // clang-format off
  // default odom covariance
  const double DEFAULT_POSE_COVARIANCE[36] = {
    1e-3, 0,    0,    0,    0,    0,
    0,    1e-3, 0,    0,    0,    0,
    0,    0,    1e6,  0,    0,    0,
    0,    0,    0,    1e6,  0,    0,
    0,    0,    0,    0,    1e6,  0,
    0,    0,    0,    0,    0,    1e3
  };

  const double DEFAULT_TWIST_COVARIANCE[36] = {
    1e-3, 0,    0,    0,    0,    0,
    0,    1e-3, 0,    0,    0,    0,
    0,    0,    1e6,  0,    0,    0,
    0,    0,    0,    1e6,  0,    0,
    0,    0,    0,    0,    1e6,  0,
    0,    0,    0,    0,    0,    1e3
  };

  const double EMPTY_POSE_COVARIANCE[36] = {
    1e-9, 0,    0,    0,    0,    0,
    0,    1e-3, 1e-9, 0,    0,    0,
    0,    0,    1e6,  0,    0,    0,
    0,    0,    0,    1e6,  0,    0,
    0,    0,    0,    0,    1e6,  0,
    0,    0,    0,    0,    0,    1e-9
  };

  const double EMPTY_TWIST_COVARIANCE[36] = {
    1e-9, 0,    0,    0,    0,    0,
    0,    1e-3, 1e-9, 0,    0,    0,
    0,    0,    1e6,  0,    0,    0,
    0,    0,    0,    1e6,  0,    0,
    0,    0,    0,    0,    1e6,  0,
    0,    0,    0,    0,    0,    1e-9
  };
  // clang-format on
};

}  // namespace robot_base
}  // namespace qrb_ros

#endif  // QRB_ROS_ROBOT_BASE__ROBOT_BASE__ODOM_CONTROLLER_HPP_
