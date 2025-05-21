// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from arussim_msgs:msg/PointXY.idl
// generated code does not contain a copyright notice
#include "arussim_msgs/msg/detail/point_xy__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
arussim_msgs__msg__PointXY__init(arussim_msgs__msg__PointXY * msg)
{
  if (!msg) {
    return false;
  }
  // x
  // y
  return true;
}

void
arussim_msgs__msg__PointXY__fini(arussim_msgs__msg__PointXY * msg)
{
  if (!msg) {
    return;
  }
  // x
  // y
}

bool
arussim_msgs__msg__PointXY__are_equal(const arussim_msgs__msg__PointXY * lhs, const arussim_msgs__msg__PointXY * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  return true;
}

bool
arussim_msgs__msg__PointXY__copy(
  const arussim_msgs__msg__PointXY * input,
  arussim_msgs__msg__PointXY * output)
{
  if (!input || !output) {
    return false;
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  return true;
}

arussim_msgs__msg__PointXY *
arussim_msgs__msg__PointXY__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arussim_msgs__msg__PointXY * msg = (arussim_msgs__msg__PointXY *)allocator.allocate(sizeof(arussim_msgs__msg__PointXY), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(arussim_msgs__msg__PointXY));
  bool success = arussim_msgs__msg__PointXY__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
arussim_msgs__msg__PointXY__destroy(arussim_msgs__msg__PointXY * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    arussim_msgs__msg__PointXY__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
arussim_msgs__msg__PointXY__Sequence__init(arussim_msgs__msg__PointXY__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arussim_msgs__msg__PointXY * data = NULL;

  if (size) {
    data = (arussim_msgs__msg__PointXY *)allocator.zero_allocate(size, sizeof(arussim_msgs__msg__PointXY), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = arussim_msgs__msg__PointXY__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        arussim_msgs__msg__PointXY__fini(&data[i - 1]);
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
arussim_msgs__msg__PointXY__Sequence__fini(arussim_msgs__msg__PointXY__Sequence * array)
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
      arussim_msgs__msg__PointXY__fini(&array->data[i]);
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

arussim_msgs__msg__PointXY__Sequence *
arussim_msgs__msg__PointXY__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arussim_msgs__msg__PointXY__Sequence * array = (arussim_msgs__msg__PointXY__Sequence *)allocator.allocate(sizeof(arussim_msgs__msg__PointXY__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = arussim_msgs__msg__PointXY__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
arussim_msgs__msg__PointXY__Sequence__destroy(arussim_msgs__msg__PointXY__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    arussim_msgs__msg__PointXY__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
arussim_msgs__msg__PointXY__Sequence__are_equal(const arussim_msgs__msg__PointXY__Sequence * lhs, const arussim_msgs__msg__PointXY__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!arussim_msgs__msg__PointXY__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
arussim_msgs__msg__PointXY__Sequence__copy(
  const arussim_msgs__msg__PointXY__Sequence * input,
  arussim_msgs__msg__PointXY__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(arussim_msgs__msg__PointXY);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    arussim_msgs__msg__PointXY * data =
      (arussim_msgs__msg__PointXY *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!arussim_msgs__msg__PointXY__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          arussim_msgs__msg__PointXY__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!arussim_msgs__msg__PointXY__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
