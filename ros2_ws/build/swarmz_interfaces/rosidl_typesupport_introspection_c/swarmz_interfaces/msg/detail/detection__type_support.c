// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from swarmz_interfaces:msg/Detection.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "swarmz_interfaces/msg/detail/detection__rosidl_typesupport_introspection_c.h"
#include "swarmz_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "swarmz_interfaces/msg/detail/detection__functions.h"
#include "swarmz_interfaces/msg/detail/detection__struct.h"


// Include directives for member types
// Member `relative_position`
#include "geometry_msgs/msg/pose.h"
// Member `relative_position`
#include "geometry_msgs/msg/detail/pose__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void swarmz_interfaces__msg__Detection__rosidl_typesupport_introspection_c__Detection_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  swarmz_interfaces__msg__Detection__init(message_memory);
}

void swarmz_interfaces__msg__Detection__rosidl_typesupport_introspection_c__Detection_fini_function(void * message_memory)
{
  swarmz_interfaces__msg__Detection__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember swarmz_interfaces__msg__Detection__rosidl_typesupport_introspection_c__Detection_message_member_array[3] = {
  {
    "vehicle_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarmz_interfaces__msg__Detection, vehicle_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "is_friend",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarmz_interfaces__msg__Detection, is_friend),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "relative_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarmz_interfaces__msg__Detection, relative_position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers swarmz_interfaces__msg__Detection__rosidl_typesupport_introspection_c__Detection_message_members = {
  "swarmz_interfaces__msg",  // message namespace
  "Detection",  // message name
  3,  // number of fields
  sizeof(swarmz_interfaces__msg__Detection),
  swarmz_interfaces__msg__Detection__rosidl_typesupport_introspection_c__Detection_message_member_array,  // message members
  swarmz_interfaces__msg__Detection__rosidl_typesupport_introspection_c__Detection_init_function,  // function to initialize message memory (memory has to be allocated)
  swarmz_interfaces__msg__Detection__rosidl_typesupport_introspection_c__Detection_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t swarmz_interfaces__msg__Detection__rosidl_typesupport_introspection_c__Detection_message_type_support_handle = {
  0,
  &swarmz_interfaces__msg__Detection__rosidl_typesupport_introspection_c__Detection_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_swarmz_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, swarmz_interfaces, msg, Detection)() {
  swarmz_interfaces__msg__Detection__rosidl_typesupport_introspection_c__Detection_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  if (!swarmz_interfaces__msg__Detection__rosidl_typesupport_introspection_c__Detection_message_type_support_handle.typesupport_identifier) {
    swarmz_interfaces__msg__Detection__rosidl_typesupport_introspection_c__Detection_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &swarmz_interfaces__msg__Detection__rosidl_typesupport_introspection_c__Detection_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
