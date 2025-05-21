// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from arussim_msgs:msg/State.idl
// generated code does not contain a copyright notice

#ifndef ARUSSIM_MSGS__MSG__DETAIL__STATE__STRUCT_H_
#define ARUSSIM_MSGS__MSG__DETAIL__STATE__STRUCT_H_

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
// Member 'wheel_speeds'
// Member 'torque'
#include "arussim_msgs/msg/detail/four_wheel_drive__struct.h"

/// Struct defined in msg/State in the package arussim_msgs.
typedef struct arussim_msgs__msg__State
{
  std_msgs__msg__Header header;
  /// m
  float x;
  /// m
  float y;
  /// radians
  float yaw;
  /// m/s
  float vx;
  /// m/s
  float vy;
  /// radians/s
  float r;
  /// m/s²
  float ax;
  /// m/s²
  float ay;
  /// radians
  float delta;
  arussim_msgs__msg__FourWheelDrive wheel_speeds;
  arussim_msgs__msg__FourWheelDrive torque;
} arussim_msgs__msg__State;

// Struct for a sequence of arussim_msgs__msg__State.
typedef struct arussim_msgs__msg__State__Sequence
{
  arussim_msgs__msg__State * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} arussim_msgs__msg__State__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARUSSIM_MSGS__MSG__DETAIL__STATE__STRUCT_H_
