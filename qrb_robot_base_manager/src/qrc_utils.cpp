// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_robot_base_manager/qrc_utils.hpp"

#include <iostream>

namespace qrb
{
namespace robot_base_manager
{
bool QrcUtils::send_message(struct qrc_pipe_s * pipe, void * data, size_t len, bool ack)
{
  if (pipe == nullptr) {
    std::cerr << "qrc pipe is null" << std::endl;
    return false;
  }

  std::cout << "QrcUtils send message, pipe id: " << (int)pipe->pipe_id
            << ", message type: " << *(int *)data << ", len: " << len << std::endl;

  auto status = qrc_write(pipe, (uint8_t *)data, len, ack);
  if (status == qrc_write_status_e::SUCCESS) {
    return true;
  }

  if (status == qrc_write_status_e::FAILED) {
    std::cerr << "qrc write failed" << std::endl;
    return false;
  }

  if (status == qrc_write_status_e::TIMEOUT) {
    std::cerr << "qrc write timeout" << std::endl;
    return false;
  }
  return false;
}

void QrcUtils::dump_message(void * data, size_t len)
{
  if (data == nullptr) {
    return;
  }
  for (size_t i = 0; i < len; i++) {
    if (i % 16 == 0 && i > 0) {
      std::cout << std::endl;
    }
    int b = static_cast<uint8_t *>(data)[i];
    std::cout << std::hex << b << " ";
  }
  std::cout << std::dec << std::endl;
}

}  // namespace robot_base_manager
}  // namespace qrb
