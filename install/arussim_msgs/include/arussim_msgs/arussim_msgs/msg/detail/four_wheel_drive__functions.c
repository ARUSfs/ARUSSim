// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from arussim_msgs:msg/FourWheelDrive.idl
// generated code does not contain a copyright notice
#include "arussim_msgs/msg/detail/four_wheel_drive__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
arussim_msgs__msg__FourWheelDrive__init(arussim_msgs__msg__FourWheelDrive * msg)
{
  if (!msg) {
    return false;
  }
  // front_right
  // front_left
  // rear_right
  // rear_left
  return true;
}

void
arussim_msgs__msg__FourWheelDrive__fini(arussim_msgs__msg__FourWheelDrive * msg)
{
  if (!msg) {
    return;
  }
  // front_right
  // front_left
  // rear_right
  // rear_left
}

bool
arussim_msgs__msg__FourWheelDrive__are_equal(const arussim_msgs__msg__FourWheelDrive * lhs, const arussim_msgs__msg__FourWheelDrive * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // front_right
  if (lhs->front_right != rhs->front_right) {
    return false;
  }
  // front_left
  if (lhs->front_left != rhs->front_left) {
    return false;
  }
  // rear_right
  if (lhs->rear_right != rhs->rear_right) {
    return false;
  }
  // rear_left
  if (lhs->rear_left != rhs->rear_left) {
    return false;
  }
  return true;
}

bool
arussim_msgs__msg__FourWheelDrive__copy(
  const arussim_msgs__msg__FourWheelDrive * input,
  arussim_msgs__msg__FourWheelDrive * output)
{
  if (!input || !output) {
    return false;
  }
  // front_right
  output->front_right = input->front_right;
  // front_left
  output->front_left = input->front_left;
  // rear_right
  output->rear_right = input->rear_right;
  // rear_left
  output->rear_left = input->rear_left;
  return true;
}

arussim_msgs__msg__FourWheelDrive *
arussim_msgs__msg__FourWheelDrive__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arussim_msgs__msg__FourWheelDrive * msg = (arussim_msgs__msg__FourWheelDrive *)allocator.allocate(sizeof(arussim_msgs__msg__FourWheelDrive), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(arussim_msgs__msg__FourWheelDrive));
  bool success = arussim_msgs__msg__FourWheelDrive__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
arussim_msgs__msg__FourWheelDrive__destroy(arussim_msgs__msg__FourWheelDrive * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    arussim_msgs__msg__FourWheelDrive__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
arussim_msgs__msg__FourWheelDrive__Sequence__init(arussim_msgs__msg__FourWheelDrive__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arussim_msgs__msg__FourWheelDrive * data = NULL;

  if (size) {
    data = (arussim_msgs__msg__FourWheelDrive *)allocator.zero_allocate(size, sizeof(arussim_msgs__msg__FourWheelDrive), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = arussim_msgs__msg__FourWheelDrive__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        arussim_msgs__msg__FourWheelDrive__fini(&data[i - 1]);
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
arussim_msgs__msg__FourWheelDrive__Sequence__fini(arussim_msgs__msg__FourWheelDrive__Sequence * array)
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
      arussim_msgs__msg__FourWheelDrive__fini(&array->data[i]);
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

arussim_msgs__msg__FourWheelDrive__Sequence *
arussim_msgs__msg__FourWheelDrive__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arussim_msgs__msg__FourWheelDrive__Sequence * array = (arussim_msgs__msg__FourWheelDrive__Sequence *)allocator.allocate(sizeof(arussim_msgs__msg__FourWheelDrive__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = arussim_msgs__msg__FourWheelDrive__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
arussim_msgs__msg__FourWheelDrive__Sequence__destroy(arussim_msgs__msg__FourWheelDrive__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    arussim_msgs__msg__FourWheelDrive__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
arussim_msgs__msg__FourWheelDrive__Sequence__are_equal(const arussim_msgs__msg__FourWheelDrive__Sequence * lhs, const arussim_msgs__msg__FourWheelDrive__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!arussim_msgs__msg__FourWheelDrive__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
arussim_msgs__msg__FourWheelDrive__Sequence__copy(
  const arussim_msgs__msg__FourWheelDrive__Sequence * input,
  arussim_msgs__msg__FourWheelDrive__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(arussim_msgs__msg__FourWheelDrive);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    arussim_msgs__msg__FourWheelDrive * data =
      (arussim_msgs__msg__FourWheelDrive *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!arussim_msgs__msg__FourWheelDrive__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          arussim_msgs__msg__FourWheelDrive__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!arussim_msgs__msg__FourWheelDrive__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
