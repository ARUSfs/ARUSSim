// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from arussim_msgs:msg/PointXY.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "arussim_msgs/msg/detail/point_xy__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace arussim_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void PointXY_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) arussim_msgs::msg::PointXY(_init);
}

void PointXY_fini_function(void * message_memory)
{
  auto typed_message = static_cast<arussim_msgs::msg::PointXY *>(message_memory);
  typed_message->~PointXY();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember PointXY_message_member_array[2] = {
  {
    "x",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arussim_msgs::msg::PointXY, x),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "y",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arussim_msgs::msg::PointXY, y),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers PointXY_message_members = {
  "arussim_msgs::msg",  // message namespace
  "PointXY",  // message name
  2,  // number of fields
  sizeof(arussim_msgs::msg::PointXY),
  PointXY_message_member_array,  // message members
  PointXY_init_function,  // function to initialize message memory (memory has to be allocated)
  PointXY_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t PointXY_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &PointXY_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace arussim_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<arussim_msgs::msg::PointXY>()
{
  return &::arussim_msgs::msg::rosidl_typesupport_introspection_cpp::PointXY_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, arussim_msgs, msg, PointXY)() {
  return &::arussim_msgs::msg::rosidl_typesupport_introspection_cpp::PointXY_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
