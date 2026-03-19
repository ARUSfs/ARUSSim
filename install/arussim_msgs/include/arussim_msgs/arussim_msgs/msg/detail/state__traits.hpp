// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from arussim_msgs:msg/State.idl
// generated code does not contain a copyright notice

#ifndef ARUSSIM_MSGS__MSG__DETAIL__STATE__TRAITS_HPP_
#define ARUSSIM_MSGS__MSG__DETAIL__STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "arussim_msgs/msg/detail/state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'wheel_speeds'
// Member 'torque'
#include "arussim_msgs/msg/detail/four_wheel_drive__traits.hpp"

namespace arussim_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const State & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << ", ";
  }

  // member: vx
  {
    out << "vx: ";
    rosidl_generator_traits::value_to_yaml(msg.vx, out);
    out << ", ";
  }

  // member: vy
  {
    out << "vy: ";
    rosidl_generator_traits::value_to_yaml(msg.vy, out);
    out << ", ";
  }

  // member: r
  {
    out << "r: ";
    rosidl_generator_traits::value_to_yaml(msg.r, out);
    out << ", ";
  }

  // member: ax
  {
    out << "ax: ";
    rosidl_generator_traits::value_to_yaml(msg.ax, out);
    out << ", ";
  }

  // member: ay
  {
    out << "ay: ";
    rosidl_generator_traits::value_to_yaml(msg.ay, out);
    out << ", ";
  }

  // member: delta
  {
    out << "delta: ";
    rosidl_generator_traits::value_to_yaml(msg.delta, out);
    out << ", ";
  }

  // member: wheel_speeds
  {
    out << "wheel_speeds: ";
    to_flow_style_yaml(msg.wheel_speeds, out);
    out << ", ";
  }

  // member: torque
  {
    out << "torque: ";
    to_flow_style_yaml(msg.torque, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const State & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << "\n";
  }

  // member: vx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vx: ";
    rosidl_generator_traits::value_to_yaml(msg.vx, out);
    out << "\n";
  }

  // member: vy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vy: ";
    rosidl_generator_traits::value_to_yaml(msg.vy, out);
    out << "\n";
  }

  // member: r
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "r: ";
    rosidl_generator_traits::value_to_yaml(msg.r, out);
    out << "\n";
  }

  // member: ax
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ax: ";
    rosidl_generator_traits::value_to_yaml(msg.ax, out);
    out << "\n";
  }

  // member: ay
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ay: ";
    rosidl_generator_traits::value_to_yaml(msg.ay, out);
    out << "\n";
  }

  // member: delta
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "delta: ";
    rosidl_generator_traits::value_to_yaml(msg.delta, out);
    out << "\n";
  }

  // member: wheel_speeds
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "wheel_speeds:\n";
    to_block_style_yaml(msg.wheel_speeds, out, indentation + 2);
  }

  // member: torque
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "torque:\n";
    to_block_style_yaml(msg.torque, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const State & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace arussim_msgs

namespace rosidl_generator_traits
{

[[deprecated("use arussim_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const arussim_msgs::msg::State & msg,
  std::ostream & out, size_t indentation = 0)
{
  arussim_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arussim_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const arussim_msgs::msg::State & msg)
{
  return arussim_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<arussim_msgs::msg::State>()
{
  return "arussim_msgs::msg::State";
}

template<>
inline const char * name<arussim_msgs::msg::State>()
{
  return "arussim_msgs/msg/State";
}

template<>
struct has_fixed_size<arussim_msgs::msg::State>
  : std::integral_constant<bool, has_fixed_size<arussim_msgs::msg::FourWheelDrive>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<arussim_msgs::msg::State>
  : std::integral_constant<bool, has_bounded_size<arussim_msgs::msg::FourWheelDrive>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<arussim_msgs::msg::State>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ARUSSIM_MSGS__MSG__DETAIL__STATE__TRAITS_HPP_
