// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from swarmz_interfaces:msg/Detection.idl
// generated code does not contain a copyright notice
#include "swarmz_interfaces/msg/detail/detection__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `relative_position`
#include "geometry_msgs/msg/detail/pose__functions.h"

bool
swarmz_interfaces__msg__Detection__init(swarmz_interfaces__msg__Detection * msg)
{
  if (!msg) {
    return false;
  }
  // vehicle_type
  // is_friend
  // relative_position
  if (!geometry_msgs__msg__Pose__init(&msg->relative_position)) {
    swarmz_interfaces__msg__Detection__fini(msg);
    return false;
  }
  return true;
}

void
swarmz_interfaces__msg__Detection__fini(swarmz_interfaces__msg__Detection * msg)
{
  if (!msg) {
    return;
  }
  // vehicle_type
  // is_friend
  // relative_position
  geometry_msgs__msg__Pose__fini(&msg->relative_position);
}

bool
swarmz_interfaces__msg__Detection__are_equal(const swarmz_interfaces__msg__Detection * lhs, const swarmz_interfaces__msg__Detection * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // vehicle_type
  if (lhs->vehicle_type != rhs->vehicle_type) {
    return false;
  }
  // is_friend
  if (lhs->is_friend != rhs->is_friend) {
    return false;
  }
  // relative_position
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->relative_position), &(rhs->relative_position)))
  {
    return false;
  }
  return true;
}

bool
swarmz_interfaces__msg__Detection__copy(
  const swarmz_interfaces__msg__Detection * input,
  swarmz_interfaces__msg__Detection * output)
{
  if (!input || !output) {
    return false;
  }
  // vehicle_type
  output->vehicle_type = input->vehicle_type;
  // is_friend
  output->is_friend = input->is_friend;
  // relative_position
  if (!geometry_msgs__msg__Pose__copy(
      &(input->relative_position), &(output->relative_position)))
  {
    return false;
  }
  return true;
}

swarmz_interfaces__msg__Detection *
swarmz_interfaces__msg__Detection__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  swarmz_interfaces__msg__Detection * msg = (swarmz_interfaces__msg__Detection *)allocator.allocate(sizeof(swarmz_interfaces__msg__Detection), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(swarmz_interfaces__msg__Detection));
  bool success = swarmz_interfaces__msg__Detection__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
swarmz_interfaces__msg__Detection__destroy(swarmz_interfaces__msg__Detection * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    swarmz_interfaces__msg__Detection__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
swarmz_interfaces__msg__Detection__Sequence__init(swarmz_interfaces__msg__Detection__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  swarmz_interfaces__msg__Detection * data = NULL;

  if (size) {
    data = (swarmz_interfaces__msg__Detection *)allocator.zero_allocate(size, sizeof(swarmz_interfaces__msg__Detection), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = swarmz_interfaces__msg__Detection__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        swarmz_interfaces__msg__Detection__fini(&data[i - 1]);
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
swarmz_interfaces__msg__Detection__Sequence__fini(swarmz_interfaces__msg__Detection__Sequence * array)
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
      swarmz_interfaces__msg__Detection__fini(&array->data[i]);
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

swarmz_interfaces__msg__Detection__Sequence *
swarmz_interfaces__msg__Detection__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  swarmz_interfaces__msg__Detection__Sequence * array = (swarmz_interfaces__msg__Detection__Sequence *)allocator.allocate(sizeof(swarmz_interfaces__msg__Detection__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = swarmz_interfaces__msg__Detection__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
swarmz_interfaces__msg__Detection__Sequence__destroy(swarmz_interfaces__msg__Detection__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    swarmz_interfaces__msg__Detection__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
swarmz_interfaces__msg__Detection__Sequence__are_equal(const swarmz_interfaces__msg__Detection__Sequence * lhs, const swarmz_interfaces__msg__Detection__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!swarmz_interfaces__msg__Detection__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
swarmz_interfaces__msg__Detection__Sequence__copy(
  const swarmz_interfaces__msg__Detection__Sequence * input,
  swarmz_interfaces__msg__Detection__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(swarmz_interfaces__msg__Detection);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    swarmz_interfaces__msg__Detection * data =
      (swarmz_interfaces__msg__Detection *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!swarmz_interfaces__msg__Detection__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          swarmz_interfaces__msg__Detection__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!swarmz_interfaces__msg__Detection__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
