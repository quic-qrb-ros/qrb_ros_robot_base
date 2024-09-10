// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROBOT_BASE_MANAGER__ERROR_HPP_
#define QRB_ROBOT_BASE_MANAGER__ERROR_HPP_

#include <functional>
#include <map>
#include <memory>
#include <string>

namespace qrb
{
namespace robot_base_manager
{
class Error
{
public:
  Error();
  int type() const;
  const std::string & message() const;

  void set_type(int type);
  void set_message(const std::string & message);

  std::string get_error_messages(int type) const;

private:
  int type_;
  std::string message_;
};

}  // namespace robot_base_manager
}  // namespace qrb

#endif  // QRB_ROBOT_BASE_MANAGER__ERROR_HPP_
