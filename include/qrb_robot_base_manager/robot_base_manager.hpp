// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROBOT_BASE_MANAGER__ROBOT_BASE_MANAGER_HPP_
#define QRB_ROBOT_BASE_MANAGER__ROBOT_BASE_MANAGER_HPP_

#include "qrb_robot_base_manager/parameter_manager.hpp"
#include "qrc_msg_management.h"

namespace qrb
{
namespace robot_base_manager
{
class RobotBaseManager
{
private:
  RobotBaseManager();
  bool init_qrc();
  bool deinit_qrc();
  bool download_parameters(const RobotBaseParameter & robot_base_param);

public:
  static RobotBaseManager & get_instance();

  bool init_robot_base_manager(const RobotBaseParameter & robot_base_param);
  void release_robot_base_manager();

private:
};

}  // namespace robot_base_manager
}  // namespace qrb

#endif  // QRB_ROBOT_BASE_MANAGER__ROBOT_BASE_MANAGER_HPP_
