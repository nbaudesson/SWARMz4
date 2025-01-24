// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from swarmz_interfaces:msg/Detections.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "swarmz_interfaces/msg/detail/detections__rosidl_typesupport_introspection_c.h"
#include "swarmz_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "swarmz_interfaces/msg/detail/detections__functions.h"
#include "swarmz_interfaces/msg/detail/detections__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `detections`
#include "swarmz_interfaces/msg/detection.h"
// Member `detections`
#include "swarmz_interfaces/msg/detail/detection__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void swarmz_interfaces__msg__Detections__rosidl_typesupport_introspection_c__Detections_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  swarmz_interfaces__msg__Detections__init(message_memory);
}

void swarmz_interfaces__msg__Detections__rosidl_typesupport_introspection_c__Detections_fini_function(void * message_memory)
{
  swarmz_interfaces__msg__Detections__fini(message_memory);
}

size_t swarmz_interfaces__msg__Detections__rosidl_typesupport_introspection_c__size_function__Detections__detections(
  const void * untyped_member)
{
  const swarmz_interfaces__msg__Detection__Sequence * member =
    (const swarmz_interfaces__msg__Detection__Sequence *)(untyped_member);
  return member->size;
}

const void * swarmz_interfaces__msg__Detections__rosidl_typesupport_introspection_c__get_const_function__Detections__detections(
  const void * untyped_member, size_t index)
{
  const swarmz_interfaces__msg__Detection__Sequence * member =
    (const swarmz_interfaces__msg__Detection__Sequence *)(untyped_member);
  return &member->data[index];
}

void * swarmz_interfaces__msg__Detections__rosidl_typesupport_introspection_c__get_function__Detections__detections(
  void * untyped_member, size_t index)
{
  swarmz_interfaces__msg__Detection__Sequence * member =
    (swarmz_interfaces__msg__Detection__Sequence *)(untyped_member);
  return &member->data[index];
}

void swarmz_interfaces__msg__Detections__rosidl_typesupport_introspection_c__fetch_function__Detections__detections(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const swarmz_interfaces__msg__Detection * item =
    ((const swarmz_interfaces__msg__Detection *)
    swarmz_interfaces__msg__Detections__rosidl_typesupport_introspection_c__get_const_function__Detections__detections(untyped_member, index));
  swarmz_interfaces__msg__Detection * value =
    (swarmz_interfaces__msg__Detection *)(untyped_value);
  *value = *item;
}

void swarmz_interfaces__msg__Detections__rosidl_typesupport_introspection_c__assign_function__Detections__detections(
  void * untyped_member, size_t index, const void * untyped_value)
{
  swarmz_interfaces__msg__Detection * item =
    ((swarmz_interfaces__msg__Detection *)
    swarmz_interfaces__msg__Detections__rosidl_typesupport_introspection_c__get_function__Detections__detections(untyped_member, index));
  const swarmz_interfaces__msg__Detection * value =
    (const swarmz_interfaces__msg__Detection *)(untyped_value);
  *item = *value;
}

bool swarmz_interfaces__msg__Detections__rosidl_typesupport_introspection_c__resize_function__Detections__detections(
  void * untyped_member, size_t size)
{
  swarmz_interfaces__msg__Detection__Sequence * member =
    (swarmz_interfaces__msg__Detection__Sequence *)(untyped_member);
  swarmz_interfaces__msg__Detection__Sequence__fini(member);
  return swarmz_interfaces__msg__Detection__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember swarmz_interfaces__msg__Detections__rosidl_typesupport_introspection_c__Detections_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarmz_interfaces__msg__Detections, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "detections",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(swarmz_interfaces__msg__Detections, detections),  // bytes offset in struct
    NULL,  // default value
    swarmz_interfaces__msg__Detections__rosidl_typesupport_introspection_c__size_function__Detections__detections,  // size() function pointer
    swarmz_interfaces__msg__Detections__rosidl_typesupport_introspection_c__get_const_function__Detections__detections,  // get_const(index) function pointer
    swarmz_interfaces__msg__Detections__rosidl_typesupport_introspection_c__get_function__Detections__detections,  // get(index) function pointer
    swarmz_interfaces__msg__Detections__rosidl_typesupport_introspection_c__fetch_function__Detections__detections,  // fetch(index, &value) function pointer
    swarmz_interfaces__msg__Detections__rosidl_typesupport_introspection_c__assign_function__Detections__detections,  // assign(index, value) function pointer
    swarmz_interfaces__msg__Detections__rosidl_typesupport_introspection_c__resize_function__Detections__detections  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers swarmz_interfaces__msg__Detections__rosidl_typesupport_introspection_c__Detections_message_members = {
  "swarmz_interfaces__msg",  // message namespace
  "Detections",  // message name
  2,  // number of fields
  sizeof(swarmz_interfaces__msg__Detections),
  swarmz_interfaces__msg__Detections__rosidl_typesupport_introspection_c__Detections_message_member_array,  // message members
  swarmz_interfaces__msg__Detections__rosidl_typesupport_introspection_c__Detections_init_function,  // function to initialize message memory (memory has to be allocated)
  swarmz_interfaces__msg__Detections__rosidl_typesupport_introspection_c__Detections_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t swarmz_interfaces__msg__Detections__rosidl_typesupport_introspection_c__Detections_message_type_support_handle = {
  0,
  &swarmz_interfaces__msg__Detections__rosidl_typesupport_introspection_c__Detections_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_swarmz_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, swarmz_interfaces, msg, Detections)() {
  swarmz_interfaces__msg__Detections__rosidl_typesupport_introspection_c__Detections_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  swarmz_interfaces__msg__Detections__rosidl_typesupport_introspection_c__Detections_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, swarmz_interfaces, msg, Detection)();
  if (!swarmz_interfaces__msg__Detections__rosidl_typesupport_introspection_c__Detections_message_type_support_handle.typesupport_identifier) {
    swarmz_interfaces__msg__Detections__rosidl_typesupport_introspection_c__Detections_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &swarmz_interfaces__msg__Detections__rosidl_typesupport_introspection_c__Detections_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
