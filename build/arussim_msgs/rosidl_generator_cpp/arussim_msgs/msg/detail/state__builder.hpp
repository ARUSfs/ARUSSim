// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from arussim_msgs:msg/State.idl
// generated code does not contain a copyright notice

#ifndef ARUSSIM_MSGS__MSG__DETAIL__STATE__BUILDER_HPP_
#define ARUSSIM_MSGS__MSG__DETAIL__STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "arussim_msgs/msg/detail/state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace arussim_msgs
{

namespace msg
{

namespace builder
{

class Init_State_torque
{
public:
  explicit Init_State_torque(::arussim_msgs::msg::State & msg)
  : msg_(msg)
  {}
  ::arussim_msgs::msg::State torque(::arussim_msgs::msg::State::_torque_type arg)
  {
    msg_.torque = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arussim_msgs::msg::State msg_;
};

class Init_State_wheel_speeds
{
public:
  explicit Init_State_wheel_speeds(::arussim_msgs::msg::State & msg)
  : msg_(msg)
  {}
  Init_State_torque wheel_speeds(::arussim_msgs::msg::State::_wheel_speeds_type arg)
  {
    msg_.wheel_speeds = std::move(arg);
    return Init_State_torque(msg_);
  }

private:
  ::arussim_msgs::msg::State msg_;
};

class Init_State_delta
{
public:
  explicit Init_State_delta(::arussim_msgs::msg::State & msg)
  : msg_(msg)
  {}
  Init_State_wheel_speeds delta(::arussim_msgs::msg::State::_delta_type arg)
  {
    msg_.delta = std::move(arg);
    return Init_State_wheel_speeds(msg_);
  }

private:
  ::arussim_msgs::msg::State msg_;
};

class Init_State_ay
{
public:
  explicit Init_State_ay(::arussim_msgs::msg::State & msg)
  : msg_(msg)
  {}
  Init_State_delta ay(::arussim_msgs::msg::State::_ay_type arg)
  {
    msg_.ay = std::move(arg);
    return Init_State_delta(msg_);
  }

private:
  ::arussim_msgs::msg::State msg_;
};

class Init_State_ax
{
public:
  explicit Init_State_ax(::arussim_msgs::msg::State & msg)
  : msg_(msg)
  {}
  Init_State_ay ax(::arussim_msgs::msg::State::_ax_type arg)
  {
    msg_.ax = std::move(arg);
    return Init_State_ay(msg_);
  }

private:
  ::arussim_msgs::msg::State msg_;
};

class Init_State_r
{
public:
  explicit Init_State_r(::arussim_msgs::msg::State & msg)
  : msg_(msg)
  {}
  Init_State_ax r(::arussim_msgs::msg::State::_r_type arg)
  {
    msg_.r = std::move(arg);
    return Init_State_ax(msg_);
  }

private:
  ::arussim_msgs::msg::State msg_;
};

class Init_State_vy
{
public:
  explicit Init_State_vy(::arussim_msgs::msg::State & msg)
  : msg_(msg)
  {}
  Init_State_r vy(::arussim_msgs::msg::State::_vy_type arg)
  {
    msg_.vy = std::move(arg);
    return Init_State_r(msg_);
  }

private:
  ::arussim_msgs::msg::State msg_;
};

class Init_State_vx
{
public:
  explicit Init_State_vx(::arussim_msgs::msg::State & msg)
  : msg_(msg)
  {}
  Init_State_vy vx(::arussim_msgs::msg::State::_vx_type arg)
  {
    msg_.vx = std::move(arg);
    return Init_State_vy(msg_);
  }

private:
  ::arussim_msgs::msg::State msg_;
};

class Init_State_yaw
{
public:
  explicit Init_State_yaw(::arussim_msgs::msg::State & msg)
  : msg_(msg)
  {}
  Init_State_vx yaw(::arussim_msgs::msg::State::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_State_vx(msg_);
  }

private:
  ::arussim_msgs::msg::State msg_;
};

class Init_State_y
{
public:
  explicit Init_State_y(::arussim_msgs::msg::State & msg)
  : msg_(msg)
  {}
  Init_State_yaw y(::arussim_msgs::msg::State::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_State_yaw(msg_);
  }

private:
  ::arussim_msgs::msg::State msg_;
};

class Init_State_x
{
public:
  explicit Init_State_x(::arussim_msgs::msg::State & msg)
  : msg_(msg)
  {}
  Init_State_y x(::arussim_msgs::msg::State::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_State_y(msg_);
  }

private:
  ::arussim_msgs::msg::State msg_;
};

class Init_State_header
{
public:
  Init_State_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_State_x header(::arussim_msgs::msg::State::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_State_x(msg_);
  }

private:
  ::arussim_msgs::msg::State msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::arussim_msgs::msg::State>()
{
  return arussim_msgs::msg::builder::Init_State_header();
}

}  // namespace arussim_msgs

#endif  // ARUSSIM_MSGS__MSG__DETAIL__STATE__BUILDER_HPP_
