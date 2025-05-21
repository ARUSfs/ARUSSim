// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from arussim_msgs:msg/Trajectory.idl
// generated code does not contain a copyright notice

#ifndef ARUSSIM_MSGS__MSG__DETAIL__TRAJECTORY__BUILDER_HPP_
#define ARUSSIM_MSGS__MSG__DETAIL__TRAJECTORY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "arussim_msgs/msg/detail/trajectory__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace arussim_msgs
{

namespace msg
{

namespace builder
{

class Init_Trajectory_acc_profile
{
public:
  explicit Init_Trajectory_acc_profile(::arussim_msgs::msg::Trajectory & msg)
  : msg_(msg)
  {}
  ::arussim_msgs::msg::Trajectory acc_profile(::arussim_msgs::msg::Trajectory::_acc_profile_type arg)
  {
    msg_.acc_profile = std::move(arg);
    return std::move(msg_);
  }

private:
  ::arussim_msgs::msg::Trajectory msg_;
};

class Init_Trajectory_speed_profile
{
public:
  explicit Init_Trajectory_speed_profile(::arussim_msgs::msg::Trajectory & msg)
  : msg_(msg)
  {}
  Init_Trajectory_acc_profile speed_profile(::arussim_msgs::msg::Trajectory::_speed_profile_type arg)
  {
    msg_.speed_profile = std::move(arg);
    return Init_Trajectory_acc_profile(msg_);
  }

private:
  ::arussim_msgs::msg::Trajectory msg_;
};

class Init_Trajectory_k
{
public:
  explicit Init_Trajectory_k(::arussim_msgs::msg::Trajectory & msg)
  : msg_(msg)
  {}
  Init_Trajectory_speed_profile k(::arussim_msgs::msg::Trajectory::_k_type arg)
  {
    msg_.k = std::move(arg);
    return Init_Trajectory_speed_profile(msg_);
  }

private:
  ::arussim_msgs::msg::Trajectory msg_;
};

class Init_Trajectory_s
{
public:
  explicit Init_Trajectory_s(::arussim_msgs::msg::Trajectory & msg)
  : msg_(msg)
  {}
  Init_Trajectory_k s(::arussim_msgs::msg::Trajectory::_s_type arg)
  {
    msg_.s = std::move(arg);
    return Init_Trajectory_k(msg_);
  }

private:
  ::arussim_msgs::msg::Trajectory msg_;
};

class Init_Trajectory_points
{
public:
  explicit Init_Trajectory_points(::arussim_msgs::msg::Trajectory & msg)
  : msg_(msg)
  {}
  Init_Trajectory_s points(::arussim_msgs::msg::Trajectory::_points_type arg)
  {
    msg_.points = std::move(arg);
    return Init_Trajectory_s(msg_);
  }

private:
  ::arussim_msgs::msg::Trajectory msg_;
};

class Init_Trajectory_header
{
public:
  Init_Trajectory_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Trajectory_points header(::arussim_msgs::msg::Trajectory::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Trajectory_points(msg_);
  }

private:
  ::arussim_msgs::msg::Trajectory msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::arussim_msgs::msg::Trajectory>()
{
  return arussim_msgs::msg::builder::Init_Trajectory_header();
}

}  // namespace arussim_msgs

#endif  // ARUSSIM_MSGS__MSG__DETAIL__TRAJECTORY__BUILDER_HPP_
