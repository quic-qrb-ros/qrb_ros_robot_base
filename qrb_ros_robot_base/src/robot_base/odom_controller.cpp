// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_robot_base/robot_base/odom_controller.hpp"

#include <fstream>
#include <utility>

#include "qrb_robot_base_manager/odom_manager.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"

using namespace std::placeholders;

namespace qrb_ros
{
namespace robot_base
{
using OdomManager = qrb::robot_base_manager::OdomManager;

OdomController::OdomController(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {}

#ifdef ODOM_DATA_TEST
static std::ofstream ofs_odom_f_;
#endif

void OdomController::run()
{
  RCLCPP_INFO(node_->get_logger(), "OdomController: start running...");

  tf_enable_ = node_->declare_parameter<bool>("motion_tf_enable", true);
  odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 30);
  if (tf_enable_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
  }

  OdomManager::get_instance().register_speed_callback(
      std::bind(&OdomController::publish_odom_velocity, this, _1, _2, _3));
  OdomManager::get_instance().register_position_callback(
      std::bind(&OdomController::publish_odom_position, this, _1, _2, _3));

#ifdef ODOM_DATA_TEST
  RCLCPP_INFO(node_->get_logger(), "OdomController: create odom.txt ...");
  ofs_odom_f_.open("odom.txt", std::ios::out);
#endif
}

bool OdomController::compare_timespec(const timespec & ts1, const timespec & ts2)
{
  if (ts1.tv_sec < ts2.tv_sec) {
    return true;
  } else if (ts1.tv_sec > ts2.tv_sec) {
    return false;
  } else {
    return ts1.tv_nsec < ts2.tv_nsec;
  }
}

void OdomController::publish_odom_velocity(float vx, float vz, timespec ts)
{
  nav_msgs::msg::Odometry odom{};
  struct timespec local_ts;

  if (0 == last_ts_.tv_sec && 0 == last_ts_.tv_nsec) {
    last_ts_ = ts;
    return;
  }

  clock_gettime(CLOCK_REALTIME, &local_ts);
  if (compare_timespec(local_ts, ts)) {
    RCLCPP_DEBUG(node_->get_logger(),
        "OdomController: timestamp over than local timestamp, will set the timestamp as local "
        "timestamp");
    ts = local_ts;
  }

  if (compare_timespec(ts, last_ts_)) {
    RCLCPP_DEBUG(node_->get_logger(),
        "OdomController: timestamp less than last timestamp, will ignore this frame");
    return;
  }

  odom.header.frame_id = "odom";
  odom.header.stamp.sec = ts.tv_sec;
  odom.header.stamp.nanosec = ts.tv_nsec;
  odom.child_frame_id = "base_footprint";

  // calculate odom twist
  double delta_ts = (ts.tv_sec - last_ts_.tv_sec) + (ts.tv_nsec - last_ts_.tv_nsec) / 1e9;
  double avg_vz = (last_vz_ + vz) / 2.0;
  double avg_angle = avg_vz * delta_ts;
  double current_odom_vx = vx * std::cos(avg_angle);
  double current_odom_vy = vx * std::sin(avg_angle);

  double odom_vx = (last_odom_vx_ + current_odom_vx) / 2.0;
  double odom_vy = (last_odom_vy_ + current_odom_vy) / 2.0;

  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = 0;
  odom.twist.twist.angular.z = vz;

  // calculate odom pose
  geometry_msgs::msg::Pose delta_pos{};
  delta_pos.position.x =
      (odom_vx * std::cos(last_angle_) - odom_vy * std::sin(last_angle_)) * delta_ts;
  delta_pos.position.y =
      (odom_vx * std::sin(last_angle_) + odom_vy * std::cos(last_angle_)) * delta_ts;
  odom.pose.pose.position.x = last_pos_.position.x + delta_pos.position.x;
  odom.pose.pose.position.y = last_pos_.position.y + delta_pos.position.y;

  tf2::Quaternion q;
  q.setRPY(0, 0, last_angle_);
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();

  if (vx == 0 && vz == 0) {
    // If the velocity is zero, it means that the error of the encoder will be relatively small,
    // and the data of the encoder will be considered more reliable
    memcpy(&odom.pose.covariance, EMPTY_POSE_COVARIANCE, sizeof(EMPTY_POSE_COVARIANCE));
    memcpy(&odom.twist.covariance, EMPTY_TWIST_COVARIANCE, sizeof(EMPTY_TWIST_COVARIANCE));
  } else {
    // If the velocity of the trolley is non-zero, considering the sliding error that may be
    // brought by the encoder in motion, the data of IMU is considered to be more reliable
    memcpy(&odom.pose.covariance, DEFAULT_POSE_COVARIANCE, sizeof(DEFAULT_POSE_COVARIANCE));
    memcpy(&odom.twist.covariance, DEFAULT_TWIST_COVARIANCE, sizeof(DEFAULT_TWIST_COVARIANCE));
  }

  if (tf_enable_) {
    // calculate transform
    geometry_msgs::msg::Transform tf{};
    tf.translation.x = odom.pose.pose.position.x;
    tf.translation.y = odom.pose.pose.position.y;
    tf.translation.z = odom.pose.pose.position.z;
    tf.rotation.x = q.x();
    tf.rotation.y = q.y();
    tf.rotation.z = q.z();
    tf.rotation.w = q.w();
    publish_tf(tf, ts);
  }

  odom_pub_->publish(odom);

  last_ts_ = ts;
  last_pos_ = odom.pose.pose;
  last_angle_ += avg_angle;
  last_odom_vx_ = current_odom_vx;
  last_odom_vy_ = current_odom_vy;
  last_vz_ = vz;

#ifdef ODOM_DATA_TEST
  // print odom for mopcap test
  ofs_odom_f_.precision(14);
  ofs_odom_f_ << ts.tv_nsec * 0.000000001 + ts.tv_sec << '\t' << odom.pose.pose.position.x << '\t'
              << odom.pose.pose.position.y << '\t' << last_angle_ << std::endl;
#endif
}

void OdomController::publish_odom_position(float px, float pz, timespec ts)
{
  std::cout << "OdomController: position mode not implement" << std::endl;
}

void OdomController::publish_tf(const geometry_msgs::msg::Transform & tf, timespec ts)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "odom";
  transform.header.stamp.sec = ts.tv_sec;
  transform.header.stamp.nanosec = ts.tv_nsec;
  transform.child_frame_id = "base_footprint";

  transform.transform.translation = tf.translation;
  transform.transform.rotation = tf.rotation;
  tf_broadcaster_->sendTransform(transform);
}

}  // namespace robot_base
}  // namespace qrb_ros
