// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from swarmz_interfaces:srv/Missile.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "swarmz_interfaces/srv/detail/missile__struct.h"
#include "swarmz_interfaces/srv/detail/missile__type_support.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace swarmz_interfaces
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _Missile_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Missile_Request_type_support_ids_t;

static const _Missile_Request_type_support_ids_t _Missile_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Missile_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Missile_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Missile_Request_type_support_symbol_names_t _Missile_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, swarmz_interfaces, srv, Missile_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, swarmz_interfaces, srv, Missile_Request)),
  }
};

typedef struct _Missile_Request_type_support_data_t
{
  void * data[2];
} _Missile_Request_type_support_data_t;

static _Missile_Request_type_support_data_t _Missile_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Missile_Request_message_typesupport_map = {
  2,
  "swarmz_interfaces",
  &_Missile_Request_message_typesupport_ids.typesupport_identifier[0],
  &_Missile_Request_message_typesupport_symbol_names.symbol_name[0],
  &_Missile_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Missile_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Missile_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace swarmz_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, swarmz_interfaces, srv, Missile_Request)() {
  return &::swarmz_interfaces::srv::rosidl_typesupport_c::Missile_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "swarmz_interfaces/srv/detail/missile__struct.h"
// already included above
// #include "swarmz_interfaces/srv/detail/missile__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace swarmz_interfaces
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _Missile_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Missile_Response_type_support_ids_t;

static const _Missile_Response_type_support_ids_t _Missile_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Missile_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Missile_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Missile_Response_type_support_symbol_names_t _Missile_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, swarmz_interfaces, srv, Missile_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, swarmz_interfaces, srv, Missile_Response)),
  }
};

typedef struct _Missile_Response_type_support_data_t
{
  void * data[2];
} _Missile_Response_type_support_data_t;

static _Missile_Response_type_support_data_t _Missile_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Missile_Response_message_typesupport_map = {
  2,
  "swarmz_interfaces",
  &_Missile_Response_message_typesupport_ids.typesupport_identifier[0],
  &_Missile_Response_message_typesupport_symbol_names.symbol_name[0],
  &_Missile_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Missile_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Missile_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace swarmz_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, swarmz_interfaces, srv, Missile_Response)() {
  return &::swarmz_interfaces::srv::rosidl_typesupport_c::Missile_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "swarmz_interfaces/srv/detail/missile__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace swarmz_interfaces
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _Missile_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Missile_type_support_ids_t;

static const _Missile_type_support_ids_t _Missile_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Missile_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Missile_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Missile_type_support_symbol_names_t _Missile_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, swarmz_interfaces, srv, Missile)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, swarmz_interfaces, srv, Missile)),
  }
};

typedef struct _Missile_type_support_data_t
{
  void * data[2];
} _Missile_type_support_data_t;

static _Missile_type_support_data_t _Missile_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Missile_service_typesupport_map = {
  2,
  "swarmz_interfaces",
  &_Missile_service_typesupport_ids.typesupport_identifier[0],
  &_Missile_service_typesupport_symbol_names.symbol_name[0],
  &_Missile_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t Missile_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Missile_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace swarmz_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, swarmz_interfaces, srv, Missile)() {
  return &::swarmz_interfaces::srv::rosidl_typesupport_c::Missile_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif
