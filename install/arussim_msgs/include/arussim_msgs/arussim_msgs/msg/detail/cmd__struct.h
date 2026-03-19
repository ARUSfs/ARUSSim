// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from arussim_msgs:msg/Cmd.idl
// generated code does not contain a copyright notice

#ifndef ARUSSIM_MSGS__MSG__DETAIL__CMD__STRUCT_H_
#define ARUSSIM_MSGS__MSG__DETAIL__CMD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/Cmd in the package arussim_msgs.
typedef struct arussim_msgs__msg__Cmd
{
  std_msgs__msg__Header header;
  float acc;
  /// radians
  float delta;
  /// radians/s
  float target_r;
} arussim_msgs__msg__Cmd;

// Struct for a sequence of arussim_msgs__msg__Cmd.
typedef struct arussim_msgs__msg__Cmd__Sequence
{
  arussim_msgs__msg__Cmd * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} arussim_msgs__msg__Cmd__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARUSSIM_MSGS__MSG__DETAIL__CMD__STRUCT_H_
