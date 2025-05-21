// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from arussim_msgs:msg/FourWheelDrive.idl
// generated code does not contain a copyright notice

#ifndef ARUSSIM_MSGS__MSG__DETAIL__FOUR_WHEEL_DRIVE__STRUCT_H_
#define ARUSSIM_MSGS__MSG__DETAIL__FOUR_WHEEL_DRIVE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/FourWheelDrive in the package arussim_msgs.
typedef struct arussim_msgs__msg__FourWheelDrive
{
  float front_right;
  float front_left;
  float rear_right;
  float rear_left;
} arussim_msgs__msg__FourWheelDrive;

// Struct for a sequence of arussim_msgs__msg__FourWheelDrive.
typedef struct arussim_msgs__msg__FourWheelDrive__Sequence
{
  arussim_msgs__msg__FourWheelDrive * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} arussim_msgs__msg__FourWheelDrive__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARUSSIM_MSGS__MSG__DETAIL__FOUR_WHEEL_DRIVE__STRUCT_H_
