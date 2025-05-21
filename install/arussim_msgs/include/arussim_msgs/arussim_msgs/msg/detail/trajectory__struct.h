// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from arussim_msgs:msg/Trajectory.idl
// generated code does not contain a copyright notice

#ifndef ARUSSIM_MSGS__MSG__DETAIL__TRAJECTORY__STRUCT_H_
#define ARUSSIM_MSGS__MSG__DETAIL__TRAJECTORY__STRUCT_H_

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
// Member 'points'
#include "arussim_msgs/msg/detail/point_xy__struct.h"
// Member 's'
// Member 'k'
// Member 'speed_profile'
// Member 'acc_profile'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/Trajectory in the package arussim_msgs.
typedef struct arussim_msgs__msg__Trajectory
{
  std_msgs__msg__Header header;
  /// points on the trajectory
  arussim_msgs__msg__PointXY__Sequence points;
  /// distance values of the points
  rosidl_runtime_c__float__Sequence s;
  /// curvature values of the points
  rosidl_runtime_c__float__Sequence k;
  /// speed profile of the trajectory
  rosidl_runtime_c__float__Sequence speed_profile;
  /// acceleration profile of the trajectory
  rosidl_runtime_c__float__Sequence acc_profile;
} arussim_msgs__msg__Trajectory;

// Struct for a sequence of arussim_msgs__msg__Trajectory.
typedef struct arussim_msgs__msg__Trajectory__Sequence
{
  arussim_msgs__msg__Trajectory * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} arussim_msgs__msg__Trajectory__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARUSSIM_MSGS__MSG__DETAIL__TRAJECTORY__STRUCT_H_
