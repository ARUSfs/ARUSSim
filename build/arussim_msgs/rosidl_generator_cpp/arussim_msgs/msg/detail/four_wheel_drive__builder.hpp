// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from arussim_msgs:msg/FourWheelDrive.idl
// generated code does not contain a copyright notice

#ifndef ARUSSIM_MSGS__MSG__DETAIL__FOUR_WHEEL_DRIVE__BUILDER_HPP_
#define ARUSSIM_MSGS__MSG__DETAIL__FOUR_WHEEL_DRIVE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "arussim_msgs/msg/detail/four_wheel_drive__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace arussim_msgs
{

namespace msg
{

namespace builder
{

class Init_FourWheelDrive_rear_left
{
public:
  explicit Init_FourWheelDrive_rear_left(::arussim_msgs::msg::FourWheelDrive & msg)
  : msg_(msg)
  {}
  ::arussim_msgs::msg::FourWheelDrive rear_left(::arussim_msgs::msg::FourWheelDrive::_rear_left_type arg)
  {
    msg_.rear_left = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arussim_msgs::msg::FourWheelDrive msg_;
};

class Init_FourWheelDrive_rear_right
{
public:
  explicit Init_FourWheelDrive_rear_right(::arussim_msgs::msg::FourWheelDrive & msg)
  : msg_(msg)
  {}
  Init_FourWheelDrive_rear_left rear_right(::arussim_msgs::msg::FourWheelDrive::_rear_right_type arg)
  {
    msg_.rear_right = std::move(arg);
    return Init_FourWheelDrive_rear_left(msg_);
  }

private:
  ::arussim_msgs::msg::FourWheelDrive msg_;
};

class Init_FourWheelDrive_front_left
{
public:
  explicit Init_FourWheelDrive_front_left(::arussim_msgs::msg::FourWheelDrive & msg)
  : msg_(msg)
  {}
  Init_FourWheelDrive_rear_right front_left(::arussim_msgs::msg::FourWheelDrive::_front_left_type arg)
  {
    msg_.front_left = std::move(arg);
    return Init_FourWheelDrive_rear_right(msg_);
  }

private:
  ::arussim_msgs::msg::FourWheelDrive msg_;
};

class Init_FourWheelDrive_front_right
{
public:
  Init_FourWheelDrive_front_right()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FourWheelDrive_front_left front_right(::arussim_msgs::msg::FourWheelDrive::_front_right_type arg)
  {
    msg_.front_right = std::move(arg);
    return Init_FourWheelDrive_front_left(msg_);
  }

private:
  ::arussim_msgs::msg::FourWheelDrive msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::arussim_msgs::msg::FourWheelDrive>()
{
  return arussim_msgs::msg::builder::Init_FourWheelDrive_front_right();
}

}  // namespace arussim_msgs

#endif  // ARUSSIM_MSGS__MSG__DETAIL__FOUR_WHEEL_DRIVE__BUILDER_HPP_
