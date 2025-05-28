// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROBOT_BASE_MANAGER__QRC_UTILS_HPP_
#define QRB_ROBOT_BASE_MANAGER__QRC_UTILS_HPP_

#include <iostream>

#include "qrc_msg_management.h"

namespace qrb
{
namespace robot_base_manager
{
class QrcUtils
{
public:
  static bool send_message(struct qrc_pipe_s * pipe, void * data, size_t len, bool ack = true);
  static void dump_message(void * data, size_t len);
};
}  // namespace robot_base_manager
}  // namespace qrb

#endif  // QRB_ROBOT_BASE_MANAGER__QRC_UTILS_HPP_