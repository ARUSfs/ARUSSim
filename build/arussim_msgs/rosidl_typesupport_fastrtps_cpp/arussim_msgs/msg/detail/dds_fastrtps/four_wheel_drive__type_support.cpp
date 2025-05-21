// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from arussim_msgs:msg/FourWheelDrive.idl
// generated code does not contain a copyright notice
#include "arussim_msgs/msg/detail/four_wheel_drive__rosidl_typesupport_fastrtps_cpp.hpp"
#include "arussim_msgs/msg/detail/four_wheel_drive__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace arussim_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_arussim_msgs
cdr_serialize(
  const arussim_msgs::msg::FourWheelDrive & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: front_right
  cdr << ros_message.front_right;
  // Member: front_left
  cdr << ros_message.front_left;
  // Member: rear_right
  cdr << ros_message.rear_right;
  // Member: rear_left
  cdr << ros_message.rear_left;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_arussim_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  arussim_msgs::msg::FourWheelDrive & ros_message)
{
  // Member: front_right
  cdr >> ros_message.front_right;

  // Member: front_left
  cdr >> ros_message.front_left;

  // Member: rear_right
  cdr >> ros_message.rear_right;

  // Member: rear_left
  cdr >> ros_message.rear_left;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_arussim_msgs
get_serialized_size(
  const arussim_msgs::msg::FourWheelDrive & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: front_right
  {
    size_t item_size = sizeof(ros_message.front_right);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: front_left
  {
    size_t item_size = sizeof(ros_message.front_left);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: rear_right
  {
    size_t item_size = sizeof(ros_message.rear_right);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: rear_left
  {
    size_t item_size = sizeof(ros_message.rear_left);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_arussim_msgs
max_serialized_size_FourWheelDrive(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: front_right
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: front_left
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: rear_right
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: rear_left
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = arussim_msgs::msg::FourWheelDrive;
    is_plain =
      (
      offsetof(DataType, rear_left) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _FourWheelDrive__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const arussim_msgs::msg::FourWheelDrive *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _FourWheelDrive__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<arussim_msgs::msg::FourWheelDrive *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _FourWheelDrive__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const arussim_msgs::msg::FourWheelDrive *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _FourWheelDrive__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_FourWheelDrive(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _FourWheelDrive__callbacks = {
  "arussim_msgs::msg",
  "FourWheelDrive",
  _FourWheelDrive__cdr_serialize,
  _FourWheelDrive__cdr_deserialize,
  _FourWheelDrive__get_serialized_size,
  _FourWheelDrive__max_serialized_size
};

static rosidl_message_type_support_t _FourWheelDrive__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_FourWheelDrive__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace arussim_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_arussim_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<arussim_msgs::msg::FourWheelDrive>()
{
  return &arussim_msgs::msg::typesupport_fastrtps_cpp::_FourWheelDrive__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, arussim_msgs, msg, FourWheelDrive)() {
  return &arussim_msgs::msg::typesupport_fastrtps_cpp::_FourWheelDrive__handle;
}

#ifdef __cplusplus
}
#endif
