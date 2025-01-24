// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from swarmz_interfaces:srv/UpdateHealth.idl
// generated code does not contain a copyright notice

#ifndef SWARMZ_INTERFACES__SRV__DETAIL__UPDATE_HEALTH__STRUCT_H_
#define SWARMZ_INTERFACES__SRV__DETAIL__UPDATE_HEALTH__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'robot_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/UpdateHealth in the package swarmz_interfaces.
typedef struct swarmz_interfaces__srv__UpdateHealth_Request
{
  rosidl_runtime_c__String robot_name;
  int32_t damage;
} swarmz_interfaces__srv__UpdateHealth_Request;

// Struct for a sequence of swarmz_interfaces__srv__UpdateHealth_Request.
typedef struct swarmz_interfaces__srv__UpdateHealth_Request__Sequence
{
  swarmz_interfaces__srv__UpdateHealth_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} swarmz_interfaces__srv__UpdateHealth_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/UpdateHealth in the package swarmz_interfaces.
typedef struct swarmz_interfaces__srv__UpdateHealth_Response
{
  bool success;
} swarmz_interfaces__srv__UpdateHealth_Response;

// Struct for a sequence of swarmz_interfaces__srv__UpdateHealth_Response.
typedef struct swarmz_interfaces__srv__UpdateHealth_Response__Sequence
{
  swarmz_interfaces__srv__UpdateHealth_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} swarmz_interfaces__srv__UpdateHealth_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SWARMZ_INTERFACES__SRV__DETAIL__UPDATE_HEALTH__STRUCT_H_
