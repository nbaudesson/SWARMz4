// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from swarmz_interfaces:msg/Detections.idl
// generated code does not contain a copyright notice

#ifndef SWARMZ_INTERFACES__MSG__DETAIL__DETECTIONS__FUNCTIONS_H_
#define SWARMZ_INTERFACES__MSG__DETAIL__DETECTIONS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "swarmz_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "swarmz_interfaces/msg/detail/detections__struct.h"

/// Initialize msg/Detections message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * swarmz_interfaces__msg__Detections
 * )) before or use
 * swarmz_interfaces__msg__Detections__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_swarmz_interfaces
bool
swarmz_interfaces__msg__Detections__init(swarmz_interfaces__msg__Detections * msg);

/// Finalize msg/Detections message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_swarmz_interfaces
void
swarmz_interfaces__msg__Detections__fini(swarmz_interfaces__msg__Detections * msg);

/// Create msg/Detections message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * swarmz_interfaces__msg__Detections__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_swarmz_interfaces
swarmz_interfaces__msg__Detections *
swarmz_interfaces__msg__Detections__create();

/// Destroy msg/Detections message.
/**
 * It calls
 * swarmz_interfaces__msg__Detections__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_swarmz_interfaces
void
swarmz_interfaces__msg__Detections__destroy(swarmz_interfaces__msg__Detections * msg);

/// Check for msg/Detections message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_swarmz_interfaces
bool
swarmz_interfaces__msg__Detections__are_equal(const swarmz_interfaces__msg__Detections * lhs, const swarmz_interfaces__msg__Detections * rhs);

/// Copy a msg/Detections message.
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
ROSIDL_GENERATOR_C_PUBLIC_swarmz_interfaces
bool
swarmz_interfaces__msg__Detections__copy(
  const swarmz_interfaces__msg__Detections * input,
  swarmz_interfaces__msg__Detections * output);

/// Initialize array of msg/Detections messages.
/**
 * It allocates the memory for the number of elements and calls
 * swarmz_interfaces__msg__Detections__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_swarmz_interfaces
bool
swarmz_interfaces__msg__Detections__Sequence__init(swarmz_interfaces__msg__Detections__Sequence * array, size_t size);

/// Finalize array of msg/Detections messages.
/**
 * It calls
 * swarmz_interfaces__msg__Detections__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_swarmz_interfaces
void
swarmz_interfaces__msg__Detections__Sequence__fini(swarmz_interfaces__msg__Detections__Sequence * array);

/// Create array of msg/Detections messages.
/**
 * It allocates the memory for the array and calls
 * swarmz_interfaces__msg__Detections__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_swarmz_interfaces
swarmz_interfaces__msg__Detections__Sequence *
swarmz_interfaces__msg__Detections__Sequence__create(size_t size);

/// Destroy array of msg/Detections messages.
/**
 * It calls
 * swarmz_interfaces__msg__Detections__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_swarmz_interfaces
void
swarmz_interfaces__msg__Detections__Sequence__destroy(swarmz_interfaces__msg__Detections__Sequence * array);

/// Check for msg/Detections message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_swarmz_interfaces
bool
swarmz_interfaces__msg__Detections__Sequence__are_equal(const swarmz_interfaces__msg__Detections__Sequence * lhs, const swarmz_interfaces__msg__Detections__Sequence * rhs);

/// Copy an array of msg/Detections messages.
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
ROSIDL_GENERATOR_C_PUBLIC_swarmz_interfaces
bool
swarmz_interfaces__msg__Detections__Sequence__copy(
  const swarmz_interfaces__msg__Detections__Sequence * input,
  swarmz_interfaces__msg__Detections__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // SWARMZ_INTERFACES__MSG__DETAIL__DETECTIONS__FUNCTIONS_H_
