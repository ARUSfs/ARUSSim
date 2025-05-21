// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from arussim_msgs:msg/PointXY.idl
// generated code does not contain a copyright notice

#ifndef ARUSSIM_MSGS__MSG__DETAIL__POINT_XY__STRUCT_H_
#define ARUSSIM_MSGS__MSG__DETAIL__POINT_XY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/PointXY in the package arussim_msgs.
typedef struct arussim_msgs__msg__PointXY
{
  /// m
  float x;
  /// m
  float y;
} arussim_msgs__msg__PointXY;

// Struct for a sequence of arussim_msgs__msg__PointXY.
typedef struct arussim_msgs__msg__PointXY__Sequence
{
  arussim_msgs__msg__PointXY * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} arussim_msgs__msg__PointXY__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARUSSIM_MSGS__MSG__DETAIL__POINT_XY__STRUCT_H_
