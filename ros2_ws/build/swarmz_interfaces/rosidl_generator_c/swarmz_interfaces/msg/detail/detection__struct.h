// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from swarmz_interfaces:msg/Detection.idl
// generated code does not contain a copyright notice

#ifndef SWARMZ_INTERFACES__MSG__DETAIL__DETECTION__STRUCT_H_
#define SWARMZ_INTERFACES__MSG__DETAIL__DETECTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'DRONE'.
enum
{
  swarmz_interfaces__msg__Detection__DRONE = 0
};

/// Constant 'SHIP'.
enum
{
  swarmz_interfaces__msg__Detection__SHIP = 1
};

// Include directives for member types
// Member 'relative_position'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in msg/Detection in the package swarmz_interfaces.
/**
  * Constants
 */
typedef struct swarmz_interfaces__msg__Detection
{
  /// Message values
  int8_t vehicle_type;
  bool is_friend;
  geometry_msgs__msg__Pose relative_position;
} swarmz_interfaces__msg__Detection;

// Struct for a sequence of swarmz_interfaces__msg__Detection.
typedef struct swarmz_interfaces__msg__Detection__Sequence
{
  swarmz_interfaces__msg__Detection * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} swarmz_interfaces__msg__Detection__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SWARMZ_INTERFACES__MSG__DETAIL__DETECTION__STRUCT_H_
