// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from arussim_msgs:msg/Cmd4WD.idl
// generated code does not contain a copyright notice

#ifndef ARUSSIM_MSGS__MSG__DETAIL__CMD4_WD__TRAITS_HPP_
#define ARUSSIM_MSGS__MSG__DETAIL__CMD4_WD__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "arussim_msgs/msg/detail/cmd4_wd__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'acc'
#include "arussim_msgs/msg/detail/four_wheel_drive__traits.hpp"

namespace arussim_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Cmd4WD & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: acc
  {
    out << "acc: ";
    to_flow_style_yaml(msg.acc, out);
    out << ", ";
  }

  // member: delta
  {
    out << "delta: ";
    rosidl_generator_traits::value_to_yaml(msg.delta, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Cmd4WD & msg,
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

  // member: acc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acc:\n";
    to_block_style_yaml(msg.acc, out, indentation + 2);
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Cmd4WD & msg, bool use_flow_style = false)
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
  const arussim_msgs::msg::Cmd4WD & msg,
  std::ostream & out, size_t indentation = 0)
{
  arussim_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arussim_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const arussim_msgs::msg::Cmd4WD & msg)
{
  return arussim_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<arussim_msgs::msg::Cmd4WD>()
{
  return "arussim_msgs::msg::Cmd4WD";
}

template<>
inline const char * name<arussim_msgs::msg::Cmd4WD>()
{
  return "arussim_msgs/msg/Cmd4WD";
}

template<>
struct has_fixed_size<arussim_msgs::msg::Cmd4WD>
  : std::integral_constant<bool, has_fixed_size<arussim_msgs::msg::FourWheelDrive>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<arussim_msgs::msg::Cmd4WD>
  : std::integral_constant<bool, has_bounded_size<arussim_msgs::msg::FourWheelDrive>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<arussim_msgs::msg::Cmd4WD>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ARUSSIM_MSGS__MSG__DETAIL__CMD4_WD__TRAITS_HPP_
