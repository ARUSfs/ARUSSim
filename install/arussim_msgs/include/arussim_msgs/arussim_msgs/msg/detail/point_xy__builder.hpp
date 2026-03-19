// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from arussim_msgs:msg/PointXY.idl
// generated code does not contain a copyright notice

#ifndef ARUSSIM_MSGS__MSG__DETAIL__POINT_XY__BUILDER_HPP_
#define ARUSSIM_MSGS__MSG__DETAIL__POINT_XY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "arussim_msgs/msg/detail/point_xy__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace arussim_msgs
{

namespace msg
{

namespace builder
{

class Init_PointXY_y
{
public:
  explicit Init_PointXY_y(::arussim_msgs::msg::PointXY & msg)
  : msg_(msg)
  {}
  ::arussim_msgs::msg::PointXY y(::arussim_msgs::msg::PointXY::_y_type arg)
  {
    msg_.y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arussim_msgs::msg::PointXY msg_;
};

class Init_PointXY_x
{
public:
  Init_PointXY_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PointXY_y x(::arussim_msgs::msg::PointXY::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_PointXY_y(msg_);
  }

private:
  ::arussim_msgs::msg::PointXY msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::arussim_msgs::msg::PointXY>()
{
  return arussim_msgs::msg::builder::Init_PointXY_x();
}

}  // namespace arussim_msgs

#endif  // ARUSSIM_MSGS__MSG__DETAIL__POINT_XY__BUILDER_HPP_
