// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROBOT_BASE_MANAGER__EXCEPTION_HPP_
#define QRB_ROBOT_BASE_MANAGER__EXCEPTION_HPP_

#include <functional>
#include <map>
#include <memory>
#include <string>

namespace qrb
{
namespace robot_base_manager
{
enum ExceptionType
{
  motion,
  charger,
  watchdog,
};

class Exception
{
public:
  Exception(ExceptionType type);

  ExceptionType type() const;
  int id() const;
  const std::string & message() const;

  void set_id(int id);
  void set_message(const std::string & message);

  std::string get_exception_messages(int id) const;

protected:
  void init_exception_messages(const std::map<int, std::string> & exception_messages);

private:
  ExceptionType type_;
  int id_;
  std::string message_;
  std::map<int, std::string> exception_messages_;
};

}  // namespace robot_base_manager
}  // namespace qrb

#endif  // QRB_ROBOT_BASE_MANAGER__EXCEPTION_HPP_
