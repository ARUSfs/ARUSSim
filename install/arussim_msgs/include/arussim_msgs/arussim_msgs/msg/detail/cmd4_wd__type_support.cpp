// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from arussim_msgs:msg/Cmd4WD.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "arussim_msgs/msg/detail/cmd4_wd__struct.hpp"
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

void Cmd4WD_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) arussim_msgs::msg::Cmd4WD(_init);
}

void Cmd4WD_fini_function(void * message_memory)
{
  auto typed_message = static_cast<arussim_msgs::msg::Cmd4WD *>(message_memory);
  typed_message->~Cmd4WD();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Cmd4WD_message_member_array[3] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arussim_msgs::msg::Cmd4WD, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "acc",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<arussim_msgs::msg::FourWheelDrive>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arussim_msgs::msg::Cmd4WD, acc),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "delta",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arussim_msgs::msg::Cmd4WD, delta),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Cmd4WD_message_members = {
  "arussim_msgs::msg",  // message namespace
  "Cmd4WD",  // message name
  3,  // number of fields
  sizeof(arussim_msgs::msg::Cmd4WD),
  Cmd4WD_message_member_array,  // message members
  Cmd4WD_init_function,  // function to initialize message memory (memory has to be allocated)
  Cmd4WD_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Cmd4WD_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Cmd4WD_message_members,
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
get_message_type_support_handle<arussim_msgs::msg::Cmd4WD>()
{
  return &::arussim_msgs::msg::rosidl_typesupport_introspection_cpp::Cmd4WD_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, arussim_msgs, msg, Cmd4WD)() {
  return &::arussim_msgs::msg::rosidl_typesupport_introspection_cpp::Cmd4WD_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
