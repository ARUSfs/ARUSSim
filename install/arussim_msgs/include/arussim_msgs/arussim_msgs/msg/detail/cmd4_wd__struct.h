// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from arussim_msgs:msg/Cmd4WD.idl
// generated code does not contain a copyright notice

#ifndef ARUSSIM_MSGS__MSG__DETAIL__CMD4_WD__STRUCT_H_
#define ARUSSIM_MSGS__MSG__DETAIL__CMD4_WD__STRUCT_H_

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
// Member 'acc'
#include "arussim_msgs/msg/detail/four_wheel_drive__struct.h"

/// Struct defined in msg/Cmd4WD in the package arussim_msgs.
typedef struct arussim_msgs__msg__Cmd4WD
{
  std_msgs__msg__Header header;
  arussim_msgs__msg__FourWheelDrive acc;
  /// radians
  float delta;
} arussim_msgs__msg__Cmd4WD;

// Struct for a sequence of arussim_msgs__msg__Cmd4WD.
typedef struct arussim_msgs__msg__Cmd4WD__Sequence
{
  arussim_msgs__msg__Cmd4WD * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} arussim_msgs__msg__Cmd4WD__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARUSSIM_MSGS__MSG__DETAIL__CMD4_WD__STRUCT_H_
