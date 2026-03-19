// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from arussim_msgs:msg/FourWheelDrive.idl
// generated code does not contain a copyright notice

#ifndef ARUSSIM_MSGS__MSG__DETAIL__FOUR_WHEEL_DRIVE__FUNCTIONS_H_
#define ARUSSIM_MSGS__MSG__DETAIL__FOUR_WHEEL_DRIVE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "arussim_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "arussim_msgs/msg/detail/four_wheel_drive__struct.h"

/// Initialize msg/FourWheelDrive message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * arussim_msgs__msg__FourWheelDrive
 * )) before or use
 * arussim_msgs__msg__FourWheelDrive__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_arussim_msgs
bool
arussim_msgs__msg__FourWheelDrive__init(arussim_msgs__msg__FourWheelDrive * msg);

/// Finalize msg/FourWheelDrive message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_arussim_msgs
void
arussim_msgs__msg__FourWheelDrive__fini(arussim_msgs__msg__FourWheelDrive * msg);

/// Create msg/FourWheelDrive message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * arussim_msgs__msg__FourWheelDrive__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_arussim_msgs
arussim_msgs__msg__FourWheelDrive *
arussim_msgs__msg__FourWheelDrive__create();

/// Destroy msg/FourWheelDrive message.
/**
 * It calls
 * arussim_msgs__msg__FourWheelDrive__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_arussim_msgs
void
arussim_msgs__msg__FourWheelDrive__destroy(arussim_msgs__msg__FourWheelDrive * msg);

/// Check for msg/FourWheelDrive message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_arussim_msgs
bool
arussim_msgs__msg__FourWheelDrive__are_equal(const arussim_msgs__msg__FourWheelDrive * lhs, const arussim_msgs__msg__FourWheelDrive * rhs);

/// Copy a msg/FourWheelDrive message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_arussim_msgs
bool
arussim_msgs__msg__FourWheelDrive__copy(
  const arussim_msgs__msg__FourWheelDrive * input,
  arussim_msgs__msg__FourWheelDrive * output);

/// Initialize array of msg/FourWheelDrive messages.
/**
 * It allocates the memory for the number of elements and calls
 * arussim_msgs__msg__FourWheelDrive__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_arussim_msgs
bool
arussim_msgs__msg__FourWheelDrive__Sequence__init(arussim_msgs__msg__FourWheelDrive__Sequence * array, size_t size);

/// Finalize array of msg/FourWheelDrive messages.
/**
 * It calls
 * arussim_msgs__msg__FourWheelDrive__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_arussim_msgs
void
arussim_msgs__msg__FourWheelDrive__Sequence__fini(arussim_msgs__msg__FourWheelDrive__Sequence * array);

/// Create array of msg/FourWheelDrive messages.
/**
 * It allocates the memory for the array and calls
 * arussim_msgs__msg__FourWheelDrive__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_arussim_msgs
arussim_msgs__msg__FourWheelDrive__Sequence *
arussim_msgs__msg__FourWheelDrive__Sequence__create(size_t size);

/// Destroy array of msg/FourWheelDrive messages.
/**
 * It calls
 * arussim_msgs__msg__FourWheelDrive__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_arussim_msgs
void
arussim_msgs__msg__FourWheelDrive__Sequence__destroy(arussim_msgs__msg__FourWheelDrive__Sequence * array);

/// Check for msg/FourWheelDrive message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_arussim_msgs
bool
arussim_msgs__msg__FourWheelDrive__Sequence__are_equal(const arussim_msgs__msg__FourWheelDrive__Sequence * lhs, const arussim_msgs__msg__FourWheelDrive__Sequence * rhs);

/// Copy an array of msg/FourWheelDrive messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_arussim_msgs
bool
arussim_msgs__msg__FourWheelDrive__Sequence__copy(
  const arussim_msgs__msg__FourWheelDrive__Sequence * input,
  arussim_msgs__msg__FourWheelDrive__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ARUSSIM_MSGS__MSG__DETAIL__FOUR_WHEEL_DRIVE__FUNCTIONS_H_
