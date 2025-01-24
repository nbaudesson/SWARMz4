// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from swarmz_interfaces:srv/Missile.idl
// generated code does not contain a copyright notice

#ifndef SWARMZ_INTERFACES__SRV__DETAIL__MISSILE__TRAITS_HPP_
#define SWARMZ_INTERFACES__SRV__DETAIL__MISSILE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "swarmz_interfaces/srv/detail/missile__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace swarmz_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Missile_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: robot_name
  {
    out << "robot_name: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_name, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Missile_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: robot_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_name: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_name, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Missile_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace swarmz_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use swarmz_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const swarmz_interfaces::srv::Missile_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  swarmz_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use swarmz_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const swarmz_interfaces::srv::Missile_Request & msg)
{
  return swarmz_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<swarmz_interfaces::srv::Missile_Request>()
{
  return "swarmz_interfaces::srv::Missile_Request";
}

template<>
inline const char * name<swarmz_interfaces::srv::Missile_Request>()
{
  return "swarmz_interfaces/srv/Missile_Request";
}

template<>
struct has_fixed_size<swarmz_interfaces::srv::Missile_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<swarmz_interfaces::srv::Missile_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<swarmz_interfaces::srv::Missile_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace swarmz_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Missile_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: has_fired
  {
    out << "has_fired: ";
    rosidl_generator_traits::value_to_yaml(msg.has_fired, out);
    out << ", ";
  }

  // member: ammo
  {
    out << "ammo: ";
    rosidl_generator_traits::value_to_yaml(msg.ammo, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Missile_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: has_fired
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "has_fired: ";
    rosidl_generator_traits::value_to_yaml(msg.has_fired, out);
    out << "\n";
  }

  // member: ammo
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ammo: ";
    rosidl_generator_traits::value_to_yaml(msg.ammo, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Missile_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace swarmz_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use swarmz_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const swarmz_interfaces::srv::Missile_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  swarmz_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use swarmz_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const swarmz_interfaces::srv::Missile_Response & msg)
{
  return swarmz_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<swarmz_interfaces::srv::Missile_Response>()
{
  return "swarmz_interfaces::srv::Missile_Response";
}

template<>
inline const char * name<swarmz_interfaces::srv::Missile_Response>()
{
  return "swarmz_interfaces/srv/Missile_Response";
}

template<>
struct has_fixed_size<swarmz_interfaces::srv::Missile_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<swarmz_interfaces::srv::Missile_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<swarmz_interfaces::srv::Missile_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<swarmz_interfaces::srv::Missile>()
{
  return "swarmz_interfaces::srv::Missile";
}

template<>
inline const char * name<swarmz_interfaces::srv::Missile>()
{
  return "swarmz_interfaces/srv/Missile";
}

template<>
struct has_fixed_size<swarmz_interfaces::srv::Missile>
  : std::integral_constant<
    bool,
    has_fixed_size<swarmz_interfaces::srv::Missile_Request>::value &&
    has_fixed_size<swarmz_interfaces::srv::Missile_Response>::value
  >
{
};

template<>
struct has_bounded_size<swarmz_interfaces::srv::Missile>
  : std::integral_constant<
    bool,
    has_bounded_size<swarmz_interfaces::srv::Missile_Request>::value &&
    has_bounded_size<swarmz_interfaces::srv::Missile_Response>::value
  >
{
};

template<>
struct is_service<swarmz_interfaces::srv::Missile>
  : std::true_type
{
};

template<>
struct is_service_request<swarmz_interfaces::srv::Missile_Request>
  : std::true_type
{
};

template<>
struct is_service_response<swarmz_interfaces::srv::Missile_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // SWARMZ_INTERFACES__SRV__DETAIL__MISSILE__TRAITS_HPP_
