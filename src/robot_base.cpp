// Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_robot_base/robot_base.hpp"

#include <memory>

#include "qrb_robot_base_manager/robot_base_manager.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

namespace qrb_ros
{
namespace robot_base
{
RobotBase::RobotBase(const rclcpp::NodeOptions & options)
  : rclcpp::Node("qrb_robot_base_manager", options)
{
}

void RobotBase::run()
{
  auto node_ptr = shared_from_this();

  parameter_ctl_ = std::make_shared<ParameterController>(node_ptr);
  parameter_ctl_->load_parameters();

  auto robot_base_param = parameter_ctl_->get_parameters();
  if (!qrb::robot_base_manager::RobotBaseManager::get_instance().init_robot_base_manager(
          robot_base_param)) {
    RCLCPP_ERROR(this->get_logger(), "RobotBase: init robot base manager failed");
    return;
  }

  sensor_ctl_ = std::make_shared<SensorController>(node_ptr);
  sensor_ctl_->run();

  client_ctl_ = std::make_shared<ClientController>(node_ptr);
  client_ctl_->run();

  motion_ctl_ = std::make_shared<MotionController>(node_ptr);
  motion_ctl_->run();

  odom_ctl_ = std::make_shared<OdomController>(node_ptr);
  odom_ctl_->run();

  emergency_ctl_ = std::make_shared<EmergencyController>(node_ptr);
  emergency_ctl_->run();

  charger_ctl_ = std::make_shared<ChargerController>(node_ptr);
  charger_ctl_->run();

  error_ctl_ = std::make_shared<ErrorController>(node_ptr);
  error_ctl_->run();
}

void RobotBase::shutdown()
{
  motion_ctl_->stop();
  rclcpp::sleep_for(std::chrono::seconds(1));
  qrb::robot_base_manager::RobotBaseManager::get_instance().release_robot_base_manager();
}

}  // namespace robot_base
}  // namespace qrb_ros

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exec;
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);

  auto robot_base_node = std::make_shared<qrb_ros::robot_base::RobotBase>(options);
  exec.add_node(robot_base_node);
  robot_base_node->run();

  exec.spin();

  robot_base_node->shutdown();
  rclcpp::shutdown();

  return 0;
}
