// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from swarmz_interfaces:msg/Detections.idl
// generated code does not contain a copyright notice
#include "swarmz_interfaces/msg/detail/detections__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `detections`
#include "swarmz_interfaces/msg/detail/detection__functions.h"

bool
swarmz_interfaces__msg__Detections__init(swarmz_interfaces__msg__Detections * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    swarmz_interfaces__msg__Detections__fini(msg);
    return false;
  }
  // detections
  if (!swarmz_interfaces__msg__Detection__Sequence__init(&msg->detections, 0)) {
    swarmz_interfaces__msg__Detections__fini(msg);
    return false;
  }
  return true;
}

void
swarmz_interfaces__msg__Detections__fini(swarmz_interfaces__msg__Detections * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // detections
  swarmz_interfaces__msg__Detection__Sequence__fini(&msg->detections);
}

bool
swarmz_interfaces__msg__Detections__are_equal(const swarmz_interfaces__msg__Detections * lhs, const swarmz_interfaces__msg__Detections * rhs)
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
  // detections
  if (!swarmz_interfaces__msg__Detection__Sequence__are_equal(
      &(lhs->detections), &(rhs->detections)))
  {
    return false;
  }
  return true;
}

bool
swarmz_interfaces__msg__Detections__copy(
  const swarmz_interfaces__msg__Detections * input,
  swarmz_interfaces__msg__Detections * output)
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
  // detections
  if (!swarmz_interfaces__msg__Detection__Sequence__copy(
      &(input->detections), &(output->detections)))
  {
    return false;
  }
  return true;
}

swarmz_interfaces__msg__Detections *
swarmz_interfaces__msg__Detections__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  swarmz_interfaces__msg__Detections * msg = (swarmz_interfaces__msg__Detections *)allocator.allocate(sizeof(swarmz_interfaces__msg__Detections), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(swarmz_interfaces__msg__Detections));
  bool success = swarmz_interfaces__msg__Detections__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
swarmz_interfaces__msg__Detections__destroy(swarmz_interfaces__msg__Detections * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    swarmz_interfaces__msg__Detections__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
swarmz_interfaces__msg__Detections__Sequence__init(swarmz_interfaces__msg__Detections__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  swarmz_interfaces__msg__Detections * data = NULL;

  if (size) {
    data = (swarmz_interfaces__msg__Detections *)allocator.zero_allocate(size, sizeof(swarmz_interfaces__msg__Detections), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = swarmz_interfaces__msg__Detections__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        swarmz_interfaces__msg__Detections__fini(&data[i - 1]);
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
swarmz_interfaces__msg__Detections__Sequence__fini(swarmz_interfaces__msg__Detections__Sequence * array)
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
      swarmz_interfaces__msg__Detections__fini(&array->data[i]);
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

swarmz_interfaces__msg__Detections__Sequence *
swarmz_interfaces__msg__Detections__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  swarmz_interfaces__msg__Detections__Sequence * array = (swarmz_interfaces__msg__Detections__Sequence *)allocator.allocate(sizeof(swarmz_interfaces__msg__Detections__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = swarmz_interfaces__msg__Detections__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
swarmz_interfaces__msg__Detections__Sequence__destroy(swarmz_interfaces__msg__Detections__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    swarmz_interfaces__msg__Detections__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
swarmz_interfaces__msg__Detections__Sequence__are_equal(const swarmz_interfaces__msg__Detections__Sequence * lhs, const swarmz_interfaces__msg__Detections__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!swarmz_interfaces__msg__Detections__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
swarmz_interfaces__msg__Detections__Sequence__copy(
  const swarmz_interfaces__msg__Detections__Sequence * input,
  swarmz_interfaces__msg__Detections__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(swarmz_interfaces__msg__Detections);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    swarmz_interfaces__msg__Detections * data =
      (swarmz_interfaces__msg__Detections *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!swarmz_interfaces__msg__Detections__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          swarmz_interfaces__msg__Detections__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!swarmz_interfaces__msg__Detections__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
