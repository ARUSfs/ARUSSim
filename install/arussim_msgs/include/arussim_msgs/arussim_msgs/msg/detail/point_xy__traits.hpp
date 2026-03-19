// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from arussim_msgs:msg/PointXY.idl
// generated code does not contain a copyright notice

#ifndef ARUSSIM_MSGS__MSG__DETAIL__POINT_XY__TRAITS_HPP_
#define ARUSSIM_MSGS__MSG__DETAIL__POINT_XY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "arussim_msgs/msg/detail/point_xy__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace arussim_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const PointXY & msg,
  std::ostream & out)
{
  out << "{";
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
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PointXY & msg,
  std::ostream & out, size_t indentation = 0)
{
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PointXY & msg, bool use_flow_style = false)
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
  const arussim_msgs::msg::PointXY & msg,
  std::ostream & out, size_t indentation = 0)
{
  arussim_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arussim_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const arussim_msgs::msg::PointXY & msg)
{
  return arussim_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<arussim_msgs::msg::PointXY>()
{
  return "arussim_msgs::msg::PointXY";
}

template<>
inline const char * name<arussim_msgs::msg::PointXY>()
{
  return "arussim_msgs/msg/PointXY";
}

template<>
struct has_fixed_size<arussim_msgs::msg::PointXY>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<arussim_msgs::msg::PointXY>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<arussim_msgs::msg::PointXY>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ARUSSIM_MSGS__MSG__DETAIL__POINT_XY__TRAITS_HPP_
