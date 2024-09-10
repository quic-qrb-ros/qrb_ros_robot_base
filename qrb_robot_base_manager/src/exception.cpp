// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_robot_base_manager/exception.hpp"

#include <utility>

namespace qrb
{
namespace robot_base_manager
{
Exception::Exception(ExceptionType type) : type_(type) {}

ExceptionType Exception::type() const
{
  return type_;
}

const std::string & Exception::message() const
{
  return message_;
}

int Exception::id() const
{
  return id_;
}

void Exception::set_id(int id)
{
  id_ = id;
}

void Exception::set_message(const std::string & message)
{
  message_ = message;
}

void Exception::init_exception_messages(const std::map<int, std::string> & exception_messages)
{
  exception_messages_ = exception_messages;
}

std::string Exception::get_exception_messages(int id) const
{
  if (exception_messages_.find(id) != exception_messages_.end()) {
    return exception_messages_.at(id);
  } else {
    return "unknown exception id: " + std::to_string(id);
  }
}
}  // namespace robot_base_manager
}  // namespace qrb
