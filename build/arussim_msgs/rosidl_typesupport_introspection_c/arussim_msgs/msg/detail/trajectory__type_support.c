// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from arussim_msgs:msg/Trajectory.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "arussim_msgs/msg/detail/trajectory__rosidl_typesupport_introspection_c.h"
#include "arussim_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "arussim_msgs/msg/detail/trajectory__functions.h"
#include "arussim_msgs/msg/detail/trajectory__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `points`
#include "arussim_msgs/msg/point_xy.h"
// Member `points`
#include "arussim_msgs/msg/detail/point_xy__rosidl_typesupport_introspection_c.h"
// Member `s`
// Member `k`
// Member `speed_profile`
// Member `acc_profile`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__Trajectory_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  arussim_msgs__msg__Trajectory__init(message_memory);
}

void arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__Trajectory_fini_function(void * message_memory)
{
  arussim_msgs__msg__Trajectory__fini(message_memory);
}

size_t arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__size_function__Trajectory__points(
  const void * untyped_member)
{
  const arussim_msgs__msg__PointXY__Sequence * member =
    (const arussim_msgs__msg__PointXY__Sequence *)(untyped_member);
  return member->size;
}

const void * arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_const_function__Trajectory__points(
  const void * untyped_member, size_t index)
{
  const arussim_msgs__msg__PointXY__Sequence * member =
    (const arussim_msgs__msg__PointXY__Sequence *)(untyped_member);
  return &member->data[index];
}

void * arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_function__Trajectory__points(
  void * untyped_member, size_t index)
{
  arussim_msgs__msg__PointXY__Sequence * member =
    (arussim_msgs__msg__PointXY__Sequence *)(untyped_member);
  return &member->data[index];
}

void arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__fetch_function__Trajectory__points(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const arussim_msgs__msg__PointXY * item =
    ((const arussim_msgs__msg__PointXY *)
    arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_const_function__Trajectory__points(untyped_member, index));
  arussim_msgs__msg__PointXY * value =
    (arussim_msgs__msg__PointXY *)(untyped_value);
  *value = *item;
}

void arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__assign_function__Trajectory__points(
  void * untyped_member, size_t index, const void * untyped_value)
{
  arussim_msgs__msg__PointXY * item =
    ((arussim_msgs__msg__PointXY *)
    arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_function__Trajectory__points(untyped_member, index));
  const arussim_msgs__msg__PointXY * value =
    (const arussim_msgs__msg__PointXY *)(untyped_value);
  *item = *value;
}

bool arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__resize_function__Trajectory__points(
  void * untyped_member, size_t size)
{
  arussim_msgs__msg__PointXY__Sequence * member =
    (arussim_msgs__msg__PointXY__Sequence *)(untyped_member);
  arussim_msgs__msg__PointXY__Sequence__fini(member);
  return arussim_msgs__msg__PointXY__Sequence__init(member, size);
}

size_t arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__size_function__Trajectory__s(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_const_function__Trajectory__s(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_function__Trajectory__s(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__fetch_function__Trajectory__s(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_const_function__Trajectory__s(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__assign_function__Trajectory__s(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_function__Trajectory__s(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__resize_function__Trajectory__s(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__size_function__Trajectory__k(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_const_function__Trajectory__k(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_function__Trajectory__k(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__fetch_function__Trajectory__k(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_const_function__Trajectory__k(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__assign_function__Trajectory__k(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_function__Trajectory__k(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__resize_function__Trajectory__k(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__size_function__Trajectory__speed_profile(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_const_function__Trajectory__speed_profile(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_function__Trajectory__speed_profile(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__fetch_function__Trajectory__speed_profile(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_const_function__Trajectory__speed_profile(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__assign_function__Trajectory__speed_profile(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_function__Trajectory__speed_profile(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__resize_function__Trajectory__speed_profile(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__size_function__Trajectory__acc_profile(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_const_function__Trajectory__acc_profile(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_function__Trajectory__acc_profile(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__fetch_function__Trajectory__acc_profile(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_const_function__Trajectory__acc_profile(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__assign_function__Trajectory__acc_profile(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_function__Trajectory__acc_profile(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__resize_function__Trajectory__acc_profile(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__Trajectory_message_member_array[6] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arussim_msgs__msg__Trajectory, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "points",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arussim_msgs__msg__Trajectory, points),  // bytes offset in struct
    NULL,  // default value
    arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__size_function__Trajectory__points,  // size() function pointer
    arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_const_function__Trajectory__points,  // get_const(index) function pointer
    arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_function__Trajectory__points,  // get(index) function pointer
    arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__fetch_function__Trajectory__points,  // fetch(index, &value) function pointer
    arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__assign_function__Trajectory__points,  // assign(index, value) function pointer
    arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__resize_function__Trajectory__points  // resize(index) function pointer
  },
  {
    "s",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arussim_msgs__msg__Trajectory, s),  // bytes offset in struct
    NULL,  // default value
    arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__size_function__Trajectory__s,  // size() function pointer
    arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_const_function__Trajectory__s,  // get_const(index) function pointer
    arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_function__Trajectory__s,  // get(index) function pointer
    arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__fetch_function__Trajectory__s,  // fetch(index, &value) function pointer
    arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__assign_function__Trajectory__s,  // assign(index, value) function pointer
    arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__resize_function__Trajectory__s  // resize(index) function pointer
  },
  {
    "k",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arussim_msgs__msg__Trajectory, k),  // bytes offset in struct
    NULL,  // default value
    arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__size_function__Trajectory__k,  // size() function pointer
    arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_const_function__Trajectory__k,  // get_const(index) function pointer
    arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_function__Trajectory__k,  // get(index) function pointer
    arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__fetch_function__Trajectory__k,  // fetch(index, &value) function pointer
    arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__assign_function__Trajectory__k,  // assign(index, value) function pointer
    arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__resize_function__Trajectory__k  // resize(index) function pointer
  },
  {
    "speed_profile",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arussim_msgs__msg__Trajectory, speed_profile),  // bytes offset in struct
    NULL,  // default value
    arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__size_function__Trajectory__speed_profile,  // size() function pointer
    arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_const_function__Trajectory__speed_profile,  // get_const(index) function pointer
    arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_function__Trajectory__speed_profile,  // get(index) function pointer
    arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__fetch_function__Trajectory__speed_profile,  // fetch(index, &value) function pointer
    arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__assign_function__Trajectory__speed_profile,  // assign(index, value) function pointer
    arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__resize_function__Trajectory__speed_profile  // resize(index) function pointer
  },
  {
    "acc_profile",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(arussim_msgs__msg__Trajectory, acc_profile),  // bytes offset in struct
    NULL,  // default value
    arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__size_function__Trajectory__acc_profile,  // size() function pointer
    arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_const_function__Trajectory__acc_profile,  // get_const(index) function pointer
    arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_function__Trajectory__acc_profile,  // get(index) function pointer
    arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__fetch_function__Trajectory__acc_profile,  // fetch(index, &value) function pointer
    arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__assign_function__Trajectory__acc_profile,  // assign(index, value) function pointer
    arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__resize_function__Trajectory__acc_profile  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__Trajectory_message_members = {
  "arussim_msgs__msg",  // message namespace
  "Trajectory",  // message name
  6,  // number of fields
  sizeof(arussim_msgs__msg__Trajectory),
  arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__Trajectory_message_member_array,  // message members
  arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__Trajectory_init_function,  // function to initialize message memory (memory has to be allocated)
  arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__Trajectory_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__Trajectory_message_type_support_handle = {
  0,
  &arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__Trajectory_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_arussim_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arussim_msgs, msg, Trajectory)() {
  arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__Trajectory_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__Trajectory_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, arussim_msgs, msg, PointXY)();
  if (!arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__Trajectory_message_type_support_handle.typesupport_identifier) {
    arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__Trajectory_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &arussim_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__Trajectory_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
