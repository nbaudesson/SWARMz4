// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from swarmz_interfaces:srv/Missile.idl
// generated code does not contain a copyright notice

#ifndef SWARMZ_INTERFACES__SRV__DETAIL__MISSILE__STRUCT_H_
#define SWARMZ_INTERFACES__SRV__DETAIL__MISSILE__STRUCT_H_

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

/// Struct defined in srv/Missile in the package swarmz_interfaces.
typedef struct swarmz_interfaces__srv__Missile_Request
{
  /// use namespace to tell to the GM who is firing
  rosidl_runtime_c__String robot_name;
} swarmz_interfaces__srv__Missile_Request;

// Struct for a sequence of swarmz_interfaces__srv__Missile_Request.
typedef struct swarmz_interfaces__srv__Missile_Request__Sequence
{
  swarmz_interfaces__srv__Missile_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} swarmz_interfaces__srv__Missile_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/Missile in the package swarmz_interfaces.
typedef struct swarmz_interfaces__srv__Missile_Response
{
  /// returns false if canon is in cooldown or has no ammo left
  bool has_fired;
  /// ammo left
  int8_t ammo;
} swarmz_interfaces__srv__Missile_Response;

// Struct for a sequence of swarmz_interfaces__srv__Missile_Response.
typedef struct swarmz_interfaces__srv__Missile_Response__Sequence
{
  swarmz_interfaces__srv__Missile_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} swarmz_interfaces__srv__Missile_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SWARMZ_INTERFACES__SRV__DETAIL__MISSILE__STRUCT_H_
