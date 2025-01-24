// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from swarmz_interfaces:srv/Missile.idl
// generated code does not contain a copyright notice

#ifndef SWARMZ_INTERFACES__SRV__DETAIL__MISSILE__BUILDER_HPP_
#define SWARMZ_INTERFACES__SRV__DETAIL__MISSILE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "swarmz_interfaces/srv/detail/missile__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace swarmz_interfaces
{

namespace srv
{

namespace builder
{

class Init_Missile_Request_robot_name
{
public:
  Init_Missile_Request_robot_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::swarmz_interfaces::srv::Missile_Request robot_name(::swarmz_interfaces::srv::Missile_Request::_robot_name_type arg)
  {
    msg_.robot_name = std::move(arg);
    return std::move(msg_);
  }

private:
  ::swarmz_interfaces::srv::Missile_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::swarmz_interfaces::srv::Missile_Request>()
{
  return swarmz_interfaces::srv::builder::Init_Missile_Request_robot_name();
}

}  // namespace swarmz_interfaces


namespace swarmz_interfaces
{

namespace srv
{

namespace builder
{

class Init_Missile_Response_ammo
{
public:
  explicit Init_Missile_Response_ammo(::swarmz_interfaces::srv::Missile_Response & msg)
  : msg_(msg)
  {}
  ::swarmz_interfaces::srv::Missile_Response ammo(::swarmz_interfaces::srv::Missile_Response::_ammo_type arg)
  {
    msg_.ammo = std::move(arg);
    return std::move(msg_);
  }

private:
  ::swarmz_interfaces::srv::Missile_Response msg_;
};

class Init_Missile_Response_has_fired
{
public:
  Init_Missile_Response_has_fired()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Missile_Response_ammo has_fired(::swarmz_interfaces::srv::Missile_Response::_has_fired_type arg)
  {
    msg_.has_fired = std::move(arg);
    return Init_Missile_Response_ammo(msg_);
  }

private:
  ::swarmz_interfaces::srv::Missile_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::swarmz_interfaces::srv::Missile_Response>()
{
  return swarmz_interfaces::srv::builder::Init_Missile_Response_has_fired();
}

}  // namespace swarmz_interfaces

#endif  // SWARMZ_INTERFACES__SRV__DETAIL__MISSILE__BUILDER_HPP_
