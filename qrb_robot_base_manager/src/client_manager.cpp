// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_robot_base_manager/client_manager.hpp"

#include <iostream>
#include <memory>
#include <mutex>
#include <utility>

#include "client_control_msg.h"
#include "qrb_robot_base_manager/qrc_utils.hpp"

namespace qrb
{
namespace robot_base_manager
{
ClientManager::ClientManager()
{
  pipe_ = qrc_get_pipe(CLIENT_PIPE);
  if (pipe_ == nullptr) {
    std::cerr << "get qrc pipe: " << CLIENT_PIPE << " failed" << std::endl;
    return;
  }
  auto callback = [](struct qrc_pipe_s *, void * data, std::size_t, bool) {
    qrc_message_handle(data);
  };
  qrc_register_message_cb(pipe_, callback);
  sleep(3);
}

ClientManager & ClientManager::get_instance()
{
  static ClientManager instance;
  return instance;
}

void ClientManager::qrc_message_handle(void * data)
{
  std::unique_lock<std::mutex> lock(get_instance().msg_mtx_);
  auto message = static_cast<struct client_msg_s *>(data);
  switch (message->msg_type) {
    case client_msg_type_e::SET_CLIENT:
      get_instance().set_client_ack_ = true;
      get_instance().client_ = static_cast<Client>(message->client);
      std::cout << "ClientManager: set client result" << ", client: " << message->client
            << std::endl;
      break;
    case client_msg_type_e::GET_CLIENT:
      get_instance().get_client_ack_ = true;
      get_instance().client_ = static_cast<Client>(message->client);
      std::cout << "ClientManager: get client result" << ", client: " << message->client
            << std::endl;
      break;
    default:
      std::cerr << "ClientManager: qrc message not valid, type: " << message->msg_type << std::endl;
      QrcUtils::dump_message(data, sizeof(client_msg_s));
      break;
  }
  lock.unlock();
  get_instance().msg_cond_.notify_all();
}

bool ClientManager::set_client(const Client & mode)
{
  client_msg_s msg{};
  msg.msg_type = client_msg_type_e::SET_CLIENT;
  msg.client = control_client_e(mode);
  if (!QrcUtils::send_message(pipe_, &msg, sizeof(msg))) {
    std::cerr << "ClientManager: set client failed" << std::endl;
    return false;
  }
  {
    std::unique_lock<std::mutex> lock(msg_mtx_);
    msg_cond_.wait(lock, [this] { return set_client_ack_; });
    set_client_ack_ = false;
  }

  if (client_ != mode) {
    return false;
  }
  return true;
}

const Client & ClientManager::get_client()
{
  client_msg_s msg{};
  msg.msg_type = client_msg_type_e::GET_CLIENT;
  if (!QrcUtils::send_message(pipe_, &msg, sizeof(msg))) {
    std::cerr << "ClientManager: get client failed" << std::endl;
  }
  {
    std::unique_lock<std::mutex> lock(msg_mtx_);
    msg_cond_.wait(lock, [this] { return get_client_ack_; });
    get_client_ack_ = false;
  }
  return client_;
}

}  // namespace robot_base_manager
}  // namespace qrb