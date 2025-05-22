// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROBOT_BASE_MANAGER__CLIENT_MANAGER_HPP_
#define QRB_ROBOT_BASE_MANAGER__CLIENT_MANAGER_HPP_

#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>

#include "qrc_msg_management.h"

namespace qrb
{
namespace robot_base_manager
{
enum class Client
{
  application,
  charger,
  remote_controller,
};

class ClientManager
{
private:
  ClientManager();

public:
  ClientManager(const ClientManager &) = delete;
  ClientManager & operator=(const ClientManager &) = delete;
  ClientManager(const ClientManager &&) = delete;
  ClientManager & operator=(const ClientManager &&) = delete;

  static ClientManager & get_instance();
  static void qrc_message_handle(void * data);

  bool set_client(const Client & mode);
  const Client & get_client();

private:
  struct qrc_pipe_s * pipe_ = nullptr;
  std::mutex msg_mtx_;
  std::condition_variable msg_cond_;
  bool set_client_ack_ = false;
  bool get_client_ack_ = false;

  Client client_{ Client::application };
};

}  // namespace robot_base_manager
}  // namespace qrb

#endif  // QRB_ROBOT_BASE_MANAGER__CLIENT_MANAGER_HPP_