// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from swarmz_interfaces:msg/Detections.idl
// generated code does not contain a copyright notice

#ifndef SWARMZ_INTERFACES__MSG__DETAIL__DETECTIONS__STRUCT_H_
#define SWARMZ_INTERFACES__MSG__DETAIL__DETECTIONS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'detections'
#include "swarmz_interfaces/msg/detail/detection__struct.h"

/// Struct defined in msg/Detections in the package swarmz_interfaces.
typedef struct swarmz_interfaces__msg__Detections
{
  std_msgs__msg__Header header;
  swarmz_interfaces__msg__Detection__Sequence detections;
} swarmz_interfaces__msg__Detections;

// Struct for a sequence of swarmz_interfaces__msg__Detections.
typedef struct swarmz_interfaces__msg__Detections__Sequence
{
  swarmz_interfaces__msg__Detections * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} swarmz_interfaces__msg__Detections__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SWARMZ_INTERFACES__MSG__DETAIL__DETECTIONS__STRUCT_H_
