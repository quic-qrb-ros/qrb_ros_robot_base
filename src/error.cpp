// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_robot_base_manager/error.hpp"

#include <utility>

#include "misc_msg.h"

namespace qrb
{
namespace robot_base_manager
{
std::map<int, std::string> error_messages{
  { static_cast<int>(robot_base_error_e::ERROR_OTHER), "ERROR_OTHER" },
  { static_cast<int>(robot_base_error_e::ERROR_WATCHDOG), "ERROR_WATCHDOG" },
  { static_cast<int>(robot_base_error_e::ERROR_MOTOR), "ERROR_MOTOR" },
  { static_cast<int>(robot_base_error_e::ERROR_CHARGER), "ERROR_CHARGER" },
};

Error::Error() {}

const std::string & Error::message() const
{
  return message_;
}

int Error::type() const
{
  return type_;
}

void Error::set_type(int type)
{
  type_ = type;
}

void Error::set_message(const std::string & message)
{
  message_ = message;
}

std::string Error::get_error_messages(int type) const
{
  if (error_messages.find(type) != error_messages.end()) {
    return error_messages.at(type);
  } else {
    return "unknown error type: " + std::to_string(type);
  }
}
}  // namespace robot_base_manager
}  // namespace qrb
