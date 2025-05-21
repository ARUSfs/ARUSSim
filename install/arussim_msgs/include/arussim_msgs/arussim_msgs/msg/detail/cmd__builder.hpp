// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from arussim_msgs:msg/Cmd.idl
// generated code does not contain a copyright notice

#ifndef ARUSSIM_MSGS__MSG__DETAIL__CMD__BUILDER_HPP_
#define ARUSSIM_MSGS__MSG__DETAIL__CMD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "arussim_msgs/msg/detail/cmd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace arussim_msgs
{

namespace msg
{

namespace builder
{

class Init_Cmd_target_r
{
public:
  explicit Init_Cmd_target_r(::arussim_msgs::msg::Cmd & msg)
  : msg_(msg)
  {}
  ::arussim_msgs::msg::Cmd target_r(::arussim_msgs::msg::Cmd::_target_r_type arg)
  {
    msg_.target_r = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arussim_msgs::msg::Cmd msg_;
};

class Init_Cmd_delta
{
public:
  explicit Init_Cmd_delta(::arussim_msgs::msg::Cmd & msg)
  : msg_(msg)
  {}
  Init_Cmd_target_r delta(::arussim_msgs::msg::Cmd::_delta_type arg)
  {
    msg_.delta = std::move(arg);
    return Init_Cmd_target_r(msg_);
  }

private:
  ::arussim_msgs::msg::Cmd msg_;
};

class Init_Cmd_acc
{
public:
  explicit Init_Cmd_acc(::arussim_msgs::msg::Cmd & msg)
  : msg_(msg)
  {}
  Init_Cmd_delta acc(::arussim_msgs::msg::Cmd::_acc_type arg)
  {
    msg_.acc = std::move(arg);
    return Init_Cmd_delta(msg_);
  }

private:
  ::arussim_msgs::msg::Cmd msg_;
};

class Init_Cmd_header
{
public:
  Init_Cmd_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Cmd_acc header(::arussim_msgs::msg::Cmd::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Cmd_acc(msg_);
  }

private:
  ::arussim_msgs::msg::Cmd msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::arussim_msgs::msg::Cmd>()
{
  return arussim_msgs::msg::builder::Init_Cmd_header();
}

}  // namespace arussim_msgs

#endif  // ARUSSIM_MSGS__MSG__DETAIL__CMD__BUILDER_HPP_
