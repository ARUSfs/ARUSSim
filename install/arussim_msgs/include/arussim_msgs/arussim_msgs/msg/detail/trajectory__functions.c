// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from arussim_msgs:msg/Trajectory.idl
// generated code does not contain a copyright notice
#include "arussim_msgs/msg/detail/trajectory__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `points`
#include "arussim_msgs/msg/detail/point_xy__functions.h"
// Member `s`
// Member `k`
// Member `speed_profile`
// Member `acc_profile`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
arussim_msgs__msg__Trajectory__init(arussim_msgs__msg__Trajectory * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    arussim_msgs__msg__Trajectory__fini(msg);
    return false;
  }
  // points
  if (!arussim_msgs__msg__PointXY__Sequence__init(&msg->points, 0)) {
    arussim_msgs__msg__Trajectory__fini(msg);
    return false;
  }
  // s
  if (!rosidl_runtime_c__float__Sequence__init(&msg->s, 0)) {
    arussim_msgs__msg__Trajectory__fini(msg);
    return false;
  }
  // k
  if (!rosidl_runtime_c__float__Sequence__init(&msg->k, 0)) {
    arussim_msgs__msg__Trajectory__fini(msg);
    return false;
  }
  // speed_profile
  if (!rosidl_runtime_c__float__Sequence__init(&msg->speed_profile, 0)) {
    arussim_msgs__msg__Trajectory__fini(msg);
    return false;
  }
  // acc_profile
  if (!rosidl_runtime_c__float__Sequence__init(&msg->acc_profile, 0)) {
    arussim_msgs__msg__Trajectory__fini(msg);
    return false;
  }
  return true;
}

void
arussim_msgs__msg__Trajectory__fini(arussim_msgs__msg__Trajectory * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // points
  arussim_msgs__msg__PointXY__Sequence__fini(&msg->points);
  // s
  rosidl_runtime_c__float__Sequence__fini(&msg->s);
  // k
  rosidl_runtime_c__float__Sequence__fini(&msg->k);
  // speed_profile
  rosidl_runtime_c__float__Sequence__fini(&msg->speed_profile);
  // acc_profile
  rosidl_runtime_c__float__Sequence__fini(&msg->acc_profile);
}

bool
arussim_msgs__msg__Trajectory__are_equal(const arussim_msgs__msg__Trajectory * lhs, const arussim_msgs__msg__Trajectory * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // points
  if (!arussim_msgs__msg__PointXY__Sequence__are_equal(
      &(lhs->points), &(rhs->points)))
  {
    return false;
  }
  // s
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->s), &(rhs->s)))
  {
    return false;
  }
  // k
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->k), &(rhs->k)))
  {
    return false;
  }
  // speed_profile
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->speed_profile), &(rhs->speed_profile)))
  {
    return false;
  }
  // acc_profile
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->acc_profile), &(rhs->acc_profile)))
  {
    return false;
  }
  return true;
}

bool
arussim_msgs__msg__Trajectory__copy(
  const arussim_msgs__msg__Trajectory * input,
  arussim_msgs__msg__Trajectory * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // points
  if (!arussim_msgs__msg__PointXY__Sequence__copy(
      &(input->points), &(output->points)))
  {
    return false;
  }
  // s
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->s), &(output->s)))
  {
    return false;
  }
  // k
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->k), &(output->k)))
  {
    return false;
  }
  // speed_profile
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->speed_profile), &(output->speed_profile)))
  {
    return false;
  }
  // acc_profile
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->acc_profile), &(output->acc_profile)))
  {
    return false;
  }
  return true;
}

arussim_msgs__msg__Trajectory *
arussim_msgs__msg__Trajectory__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arussim_msgs__msg__Trajectory * msg = (arussim_msgs__msg__Trajectory *)allocator.allocate(sizeof(arussim_msgs__msg__Trajectory), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(arussim_msgs__msg__Trajectory));
  bool success = arussim_msgs__msg__Trajectory__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
arussim_msgs__msg__Trajectory__destroy(arussim_msgs__msg__Trajectory * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    arussim_msgs__msg__Trajectory__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
arussim_msgs__msg__Trajectory__Sequence__init(arussim_msgs__msg__Trajectory__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arussim_msgs__msg__Trajectory * data = NULL;

  if (size) {
    data = (arussim_msgs__msg__Trajectory *)allocator.zero_allocate(size, sizeof(arussim_msgs__msg__Trajectory), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = arussim_msgs__msg__Trajectory__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        arussim_msgs__msg__Trajectory__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
arussim_msgs__msg__Trajectory__Sequence__fini(arussim_msgs__msg__Trajectory__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      arussim_msgs__msg__Trajectory__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

arussim_msgs__msg__Trajectory__Sequence *
arussim_msgs__msg__Trajectory__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arussim_msgs__msg__Trajectory__Sequence * array = (arussim_msgs__msg__Trajectory__Sequence *)allocator.allocate(sizeof(arussim_msgs__msg__Trajectory__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = arussim_msgs__msg__Trajectory__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
arussim_msgs__msg__Trajectory__Sequence__destroy(arussim_msgs__msg__Trajectory__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    arussim_msgs__msg__Trajectory__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
arussim_msgs__msg__Trajectory__Sequence__are_equal(const arussim_msgs__msg__Trajectory__Sequence * lhs, const arussim_msgs__msg__Trajectory__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!arussim_msgs__msg__Trajectory__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
arussim_msgs__msg__Trajectory__Sequence__copy(
  const arussim_msgs__msg__Trajectory__Sequence * input,
  arussim_msgs__msg__Trajectory__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(arussim_msgs__msg__Trajectory);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    arussim_msgs__msg__Trajectory * data =
      (arussim_msgs__msg__Trajectory *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!arussim_msgs__msg__Trajectory__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          arussim_msgs__msg__Trajectory__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!arussim_msgs__msg__Trajectory__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
