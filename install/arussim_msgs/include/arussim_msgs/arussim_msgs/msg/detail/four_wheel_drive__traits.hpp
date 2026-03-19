// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from arussim_msgs:msg/FourWheelDrive.idl
// generated code does not contain a copyright notice

#ifndef ARUSSIM_MSGS__MSG__DETAIL__FOUR_WHEEL_DRIVE__TRAITS_HPP_
#define ARUSSIM_MSGS__MSG__DETAIL__FOUR_WHEEL_DRIVE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "arussim_msgs/msg/detail/four_wheel_drive__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace arussim_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const FourWheelDrive & msg,
  std::ostream & out)
{
  out << "{";
  // member: front_right
  {
    out << "front_right: ";
    rosidl_generator_traits::value_to_yaml(msg.front_right, out);
    out << ", ";
  }

  // member: front_left
  {
    out << "front_left: ";
    rosidl_generator_traits::value_to_yaml(msg.front_left, out);
    out << ", ";
  }

  // member: rear_right
  {
    out << "rear_right: ";
    rosidl_generator_traits::value_to_yaml(msg.rear_right, out);
    out << ", ";
  }

  // member: rear_left
  {
    out << "rear_left: ";
    rosidl_generator_traits::value_to_yaml(msg.rear_left, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FourWheelDrive & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: front_right
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "front_right: ";
    rosidl_generator_traits::value_to_yaml(msg.front_right, out);
    out << "\n";
  }

  // member: front_left
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "front_left: ";
    rosidl_generator_traits::value_to_yaml(msg.front_left, out);
    out << "\n";
  }

  // member: rear_right
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rear_right: ";
    rosidl_generator_traits::value_to_yaml(msg.rear_right, out);
    out << "\n";
  }

  // member: rear_left
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rear_left: ";
    rosidl_generator_traits::value_to_yaml(msg.rear_left, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FourWheelDrive & msg, bool use_flow_style = false)
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
  const arussim_msgs::msg::FourWheelDrive & msg,
  std::ostream & out, size_t indentation = 0)
{
  arussim_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use arussim_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const arussim_msgs::msg::FourWheelDrive & msg)
{
  return arussim_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<arussim_msgs::msg::FourWheelDrive>()
{
  return "arussim_msgs::msg::FourWheelDrive";
}

template<>
inline const char * name<arussim_msgs::msg::FourWheelDrive>()
{
  return "arussim_msgs/msg/FourWheelDrive";
}

template<>
struct has_fixed_size<arussim_msgs::msg::FourWheelDrive>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<arussim_msgs::msg::FourWheelDrive>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<arussim_msgs::msg::FourWheelDrive>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ARUSSIM_MSGS__MSG__DETAIL__FOUR_WHEEL_DRIVE__TRAITS_HPP_
