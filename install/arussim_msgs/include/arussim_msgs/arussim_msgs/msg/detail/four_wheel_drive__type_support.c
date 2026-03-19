// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from arussim_msgs:msg/FourWheelDrive.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "arussim_msgs/msg/detail/four_wheel_drive__rosidl_typesupport_introspection_c.h"
#include "arussim_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "arussim_msgs/msg/detail/four_wheel_drive__functions.h"
#include "arussim_msgs/msg/detail/four_wheel_drive__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void arussim_msgs__msg__FourWheelDrive__rosidl_typesupport_introspection_c__FourWheelDrive_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  arussim_msgs__msg__FourWheelDrive__init(message_memory);
}

void arussim_msgs__msg__FourWheelDrive__rosidl_typesupport_introspection_c__FourWheelDrive_fini_function(void * message_memory)
{
  arussim_msgs__msg__FourWheelDrive__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember arussim_msgs__msg__FourWheelDrive__rosidl_typesupport_introspection_c__FourWheelDrive_message_member_array[4] = {
  {
    "front_right",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arussim_msgs__msg__FourWheelDrive, front_right),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "front_left",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arussim_msgs__msg__FourWheelDrive, front_left),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "rear_right",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arussim_msgs__msg__FourWheelDrive, rear_right),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "rear_left",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arussim_msgs__msg__FourWheelDrive, rear_left),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers arussim_msgs__msg__FourWheelDrive__rosidl_typesupport_introspection_c__FourWheelDrive_message_members = {
  "arussim_msgs__msg",  // message namespace
  "FourWheelDrive",  // message name
  4,  // number of fields
  sizeof(arussim_msgs__msg__FourWheelDrive),
  arussim_msgs__msg__FourWheelDrive__rosidl_typesupport_introspection_c__FourWheelDrive_message_member_array,  // message members
  arussim_msgs__msg__FourWheelDrive__rosidl_typesupport_introspection_c__FourWheelDrive_init_function,  // function to initialize message memory (memory has to be allocated)
  arussim_msgs__msg__FourWheelDrive__rosidl_typesupport_introspection_c__FourWheelDrive_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t arussim_msgs__msg__FourWheelDrive__rosidl_typesupport_introspection_c__FourWheelDrive_message_type_support_handle = {
  0,
  &arussim_msgs__msg__FourWheelDrive__rosidl_typesupport_introspection_c__FourWheelDrive_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_arussim_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arussim_msgs, msg, FourWheelDrive)() {
  if (!arussim_msgs__msg__FourWheelDrive__rosidl_typesupport_introspection_c__FourWheelDrive_message_type_support_handle.typesupport_identifier) {
    arussim_msgs__msg__FourWheelDrive__rosidl_typesupport_introspection_c__FourWheelDrive_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &arussim_msgs__msg__FourWheelDrive__rosidl_typesupport_introspection_c__FourWheelDrive_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
