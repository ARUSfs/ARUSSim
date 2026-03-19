// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from arussim_msgs:msg/Cmd4WD.idl
// generated code does not contain a copyright notice
#include "arussim_msgs/msg/detail/cmd4_wd__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `acc`
#include "arussim_msgs/msg/detail/four_wheel_drive__functions.h"

bool
arussim_msgs__msg__Cmd4WD__init(arussim_msgs__msg__Cmd4WD * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    arussim_msgs__msg__Cmd4WD__fini(msg);
    return false;
  }
  // acc
  if (!arussim_msgs__msg__FourWheelDrive__init(&msg->acc)) {
    arussim_msgs__msg__Cmd4WD__fini(msg);
    return false;
  }
  // delta
  return true;
}

void
arussim_msgs__msg__Cmd4WD__fini(arussim_msgs__msg__Cmd4WD * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // acc
  arussim_msgs__msg__FourWheelDrive__fini(&msg->acc);
  // delta
}

bool
arussim_msgs__msg__Cmd4WD__are_equal(const arussim_msgs__msg__Cmd4WD * lhs, const arussim_msgs__msg__Cmd4WD * rhs)
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
  // acc
  if (!arussim_msgs__msg__FourWheelDrive__are_equal(
      &(lhs->acc), &(rhs->acc)))
  {
    return false;
  }
  // delta
  if (lhs->delta != rhs->delta) {
    return false;
  }
  return true;
}

bool
arussim_msgs__msg__Cmd4WD__copy(
  const arussim_msgs__msg__Cmd4WD * input,
  arussim_msgs__msg__Cmd4WD * output)
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
  // acc
  if (!arussim_msgs__msg__FourWheelDrive__copy(
      &(input->acc), &(output->acc)))
  {
    return false;
  }
  // delta
  output->delta = input->delta;
  return true;
}

arussim_msgs__msg__Cmd4WD *
arussim_msgs__msg__Cmd4WD__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arussim_msgs__msg__Cmd4WD * msg = (arussim_msgs__msg__Cmd4WD *)allocator.allocate(sizeof(arussim_msgs__msg__Cmd4WD), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(arussim_msgs__msg__Cmd4WD));
  bool success = arussim_msgs__msg__Cmd4WD__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
arussim_msgs__msg__Cmd4WD__destroy(arussim_msgs__msg__Cmd4WD * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    arussim_msgs__msg__Cmd4WD__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
arussim_msgs__msg__Cmd4WD__Sequence__init(arussim_msgs__msg__Cmd4WD__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arussim_msgs__msg__Cmd4WD * data = NULL;

  if (size) {
    data = (arussim_msgs__msg__Cmd4WD *)allocator.zero_allocate(size, sizeof(arussim_msgs__msg__Cmd4WD), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = arussim_msgs__msg__Cmd4WD__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        arussim_msgs__msg__Cmd4WD__fini(&data[i - 1]);
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
arussim_msgs__msg__Cmd4WD__Sequence__fini(arussim_msgs__msg__Cmd4WD__Sequence * array)
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
      arussim_msgs__msg__Cmd4WD__fini(&array->data[i]);
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

arussim_msgs__msg__Cmd4WD__Sequence *
arussim_msgs__msg__Cmd4WD__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  arussim_msgs__msg__Cmd4WD__Sequence * array = (arussim_msgs__msg__Cmd4WD__Sequence *)allocator.allocate(sizeof(arussim_msgs__msg__Cmd4WD__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = arussim_msgs__msg__Cmd4WD__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
arussim_msgs__msg__Cmd4WD__Sequence__destroy(arussim_msgs__msg__Cmd4WD__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    arussim_msgs__msg__Cmd4WD__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
arussim_msgs__msg__Cmd4WD__Sequence__are_equal(const arussim_msgs__msg__Cmd4WD__Sequence * lhs, const arussim_msgs__msg__Cmd4WD__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!arussim_msgs__msg__Cmd4WD__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
arussim_msgs__msg__Cmd4WD__Sequence__copy(
  const arussim_msgs__msg__Cmd4WD__Sequence * input,
  arussim_msgs__msg__Cmd4WD__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(arussim_msgs__msg__Cmd4WD);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    arussim_msgs__msg__Cmd4WD * data =
      (arussim_msgs__msg__Cmd4WD *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!arussim_msgs__msg__Cmd4WD__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          arussim_msgs__msg__Cmd4WD__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!arussim_msgs__msg__Cmd4WD__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
