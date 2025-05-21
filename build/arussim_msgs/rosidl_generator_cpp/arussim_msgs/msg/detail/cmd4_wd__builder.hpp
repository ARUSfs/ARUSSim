// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from arussim_msgs:msg/Cmd4WD.idl
// generated code does not contain a copyright notice

#ifndef ARUSSIM_MSGS__MSG__DETAIL__CMD4_WD__BUILDER_HPP_
#define ARUSSIM_MSGS__MSG__DETAIL__CMD4_WD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "arussim_msgs/msg/detail/cmd4_wd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace arussim_msgs
{

namespace msg
{

namespace builder
{

class Init_Cmd4WD_delta
{
public:
  explicit Init_Cmd4WD_delta(::arussim_msgs::msg::Cmd4WD & msg)
  : msg_(msg)
  {}
  ::arussim_msgs::msg::Cmd4WD delta(::arussim_msgs::msg::Cmd4WD::_delta_type arg)
  {
    msg_.delta = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arussim_msgs::msg::Cmd4WD msg_;
};

class Init_Cmd4WD_acc
{
public:
  explicit Init_Cmd4WD_acc(::arussim_msgs::msg::Cmd4WD & msg)
  : msg_(msg)
  {}
  Init_Cmd4WD_delta acc(::arussim_msgs::msg::Cmd4WD::_acc_type arg)
  {
    msg_.acc = std::move(arg);
    return Init_Cmd4WD_delta(msg_);
  }

private:
  ::arussim_msgs::msg::Cmd4WD msg_;
};

class Init_Cmd4WD_header
{
public:
  Init_Cmd4WD_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Cmd4WD_acc header(::arussim_msgs::msg::Cmd4WD::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Cmd4WD_acc(msg_);
  }

private:
  ::arussim_msgs::msg::Cmd4WD msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::arussim_msgs::msg::Cmd4WD>()
{
  return arussim_msgs::msg::builder::Init_Cmd4WD_header();
}

}  // namespace arussim_msgs

#endif  // ARUSSIM_MSGS__MSG__DETAIL__CMD4_WD__BUILDER_HPP_
