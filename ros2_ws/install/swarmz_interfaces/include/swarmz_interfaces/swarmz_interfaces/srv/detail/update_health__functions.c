// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from swarmz_interfaces:srv/UpdateHealth.idl
// generated code does not contain a copyright notice
#include "swarmz_interfaces/srv/detail/update_health__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `robot_name`
#include "rosidl_runtime_c/string_functions.h"

bool
swarmz_interfaces__srv__UpdateHealth_Request__init(swarmz_interfaces__srv__UpdateHealth_Request * msg)
{
  if (!msg) {
    return false;
  }
  // robot_name
  if (!rosidl_runtime_c__String__init(&msg->robot_name)) {
    swarmz_interfaces__srv__UpdateHealth_Request__fini(msg);
    return false;
  }
  // damage
  return true;
}

void
swarmz_interfaces__srv__UpdateHealth_Request__fini(swarmz_interfaces__srv__UpdateHealth_Request * msg)
{
  if (!msg) {
    return;
  }
  // robot_name
  rosidl_runtime_c__String__fini(&msg->robot_name);
  // damage
}

bool
swarmz_interfaces__srv__UpdateHealth_Request__are_equal(const swarmz_interfaces__srv__UpdateHealth_Request * lhs, const swarmz_interfaces__srv__UpdateHealth_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // robot_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->robot_name), &(rhs->robot_name)))
  {
    return false;
  }
  // damage
  if (lhs->damage != rhs->damage) {
    return false;
  }
  return true;
}

bool
swarmz_interfaces__srv__UpdateHealth_Request__copy(
  const swarmz_interfaces__srv__UpdateHealth_Request * input,
  swarmz_interfaces__srv__UpdateHealth_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // robot_name
  if (!rosidl_runtime_c__String__copy(
      &(input->robot_name), &(output->robot_name)))
  {
    return false;
  }
  // damage
  output->damage = input->damage;
  return true;
}

swarmz_interfaces__srv__UpdateHealth_Request *
swarmz_interfaces__srv__UpdateHealth_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  swarmz_interfaces__srv__UpdateHealth_Request * msg = (swarmz_interfaces__srv__UpdateHealth_Request *)allocator.allocate(sizeof(swarmz_interfaces__srv__UpdateHealth_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(swarmz_interfaces__srv__UpdateHealth_Request));
  bool success = swarmz_interfaces__srv__UpdateHealth_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
swarmz_interfaces__srv__UpdateHealth_Request__destroy(swarmz_interfaces__srv__UpdateHealth_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    swarmz_interfaces__srv__UpdateHealth_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
swarmz_interfaces__srv__UpdateHealth_Request__Sequence__init(swarmz_interfaces__srv__UpdateHealth_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  swarmz_interfaces__srv__UpdateHealth_Request * data = NULL;

  if (size) {
    data = (swarmz_interfaces__srv__UpdateHealth_Request *)allocator.zero_allocate(size, sizeof(swarmz_interfaces__srv__UpdateHealth_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = swarmz_interfaces__srv__UpdateHealth_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        swarmz_interfaces__srv__UpdateHealth_Request__fini(&data[i - 1]);
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
swarmz_interfaces__srv__UpdateHealth_Request__Sequence__fini(swarmz_interfaces__srv__UpdateHealth_Request__Sequence * array)
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
      swarmz_interfaces__srv__UpdateHealth_Request__fini(&array->data[i]);
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

swarmz_interfaces__srv__UpdateHealth_Request__Sequence *
swarmz_interfaces__srv__UpdateHealth_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  swarmz_interfaces__srv__UpdateHealth_Request__Sequence * array = (swarmz_interfaces__srv__UpdateHealth_Request__Sequence *)allocator.allocate(sizeof(swarmz_interfaces__srv__UpdateHealth_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = swarmz_interfaces__srv__UpdateHealth_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
swarmz_interfaces__srv__UpdateHealth_Request__Sequence__destroy(swarmz_interfaces__srv__UpdateHealth_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    swarmz_interfaces__srv__UpdateHealth_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
swarmz_interfaces__srv__UpdateHealth_Request__Sequence__are_equal(const swarmz_interfaces__srv__UpdateHealth_Request__Sequence * lhs, const swarmz_interfaces__srv__UpdateHealth_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!swarmz_interfaces__srv__UpdateHealth_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
swarmz_interfaces__srv__UpdateHealth_Request__Sequence__copy(
  const swarmz_interfaces__srv__UpdateHealth_Request__Sequence * input,
  swarmz_interfaces__srv__UpdateHealth_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(swarmz_interfaces__srv__UpdateHealth_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    swarmz_interfaces__srv__UpdateHealth_Request * data =
      (swarmz_interfaces__srv__UpdateHealth_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!swarmz_interfaces__srv__UpdateHealth_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          swarmz_interfaces__srv__UpdateHealth_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!swarmz_interfaces__srv__UpdateHealth_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
swarmz_interfaces__srv__UpdateHealth_Response__init(swarmz_interfaces__srv__UpdateHealth_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
swarmz_interfaces__srv__UpdateHealth_Response__fini(swarmz_interfaces__srv__UpdateHealth_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
}

bool
swarmz_interfaces__srv__UpdateHealth_Response__are_equal(const swarmz_interfaces__srv__UpdateHealth_Response * lhs, const swarmz_interfaces__srv__UpdateHealth_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
swarmz_interfaces__srv__UpdateHealth_Response__copy(
  const swarmz_interfaces__srv__UpdateHealth_Response * input,
  swarmz_interfaces__srv__UpdateHealth_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

swarmz_interfaces__srv__UpdateHealth_Response *
swarmz_interfaces__srv__UpdateHealth_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  swarmz_interfaces__srv__UpdateHealth_Response * msg = (swarmz_interfaces__srv__UpdateHealth_Response *)allocator.allocate(sizeof(swarmz_interfaces__srv__UpdateHealth_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(swarmz_interfaces__srv__UpdateHealth_Response));
  bool success = swarmz_interfaces__srv__UpdateHealth_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
swarmz_interfaces__srv__UpdateHealth_Response__destroy(swarmz_interfaces__srv__UpdateHealth_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    swarmz_interfaces__srv__UpdateHealth_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
swarmz_interfaces__srv__UpdateHealth_Response__Sequence__init(swarmz_interfaces__srv__UpdateHealth_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  swarmz_interfaces__srv__UpdateHealth_Response * data = NULL;

  if (size) {
    data = (swarmz_interfaces__srv__UpdateHealth_Response *)allocator.zero_allocate(size, sizeof(swarmz_interfaces__srv__UpdateHealth_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = swarmz_interfaces__srv__UpdateHealth_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        swarmz_interfaces__srv__UpdateHealth_Response__fini(&data[i - 1]);
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
swarmz_interfaces__srv__UpdateHealth_Response__Sequence__fini(swarmz_interfaces__srv__UpdateHealth_Response__Sequence * array)
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
      swarmz_interfaces__srv__UpdateHealth_Response__fini(&array->data[i]);
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

swarmz_interfaces__srv__UpdateHealth_Response__Sequence *
swarmz_interfaces__srv__UpdateHealth_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  swarmz_interfaces__srv__UpdateHealth_Response__Sequence * array = (swarmz_interfaces__srv__UpdateHealth_Response__Sequence *)allocator.allocate(sizeof(swarmz_interfaces__srv__UpdateHealth_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = swarmz_interfaces__srv__UpdateHealth_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
swarmz_interfaces__srv__UpdateHealth_Response__Sequence__destroy(swarmz_interfaces__srv__UpdateHealth_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    swarmz_interfaces__srv__UpdateHealth_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
swarmz_interfaces__srv__UpdateHealth_Response__Sequence__are_equal(const swarmz_interfaces__srv__UpdateHealth_Response__Sequence * lhs, const swarmz_interfaces__srv__UpdateHealth_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!swarmz_interfaces__srv__UpdateHealth_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
swarmz_interfaces__srv__UpdateHealth_Response__Sequence__copy(
  const swarmz_interfaces__srv__UpdateHealth_Response__Sequence * input,
  swarmz_interfaces__srv__UpdateHealth_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(swarmz_interfaces__srv__UpdateHealth_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    swarmz_interfaces__srv__UpdateHealth_Response * data =
      (swarmz_interfaces__srv__UpdateHealth_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!swarmz_interfaces__srv__UpdateHealth_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          swarmz_interfaces__srv__UpdateHealth_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!swarmz_interfaces__srv__UpdateHealth_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
