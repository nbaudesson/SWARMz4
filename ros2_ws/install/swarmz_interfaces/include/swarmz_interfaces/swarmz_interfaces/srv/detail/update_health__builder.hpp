// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from swarmz_interfaces:srv/UpdateHealth.idl
// generated code does not contain a copyright notice

#ifndef SWARMZ_INTERFACES__SRV__DETAIL__UPDATE_HEALTH__BUILDER_HPP_
#define SWARMZ_INTERFACES__SRV__DETAIL__UPDATE_HEALTH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "swarmz_interfaces/srv/detail/update_health__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace swarmz_interfaces
{

namespace srv
{

namespace builder
{

class Init_UpdateHealth_Request_damage
{
public:
  explicit Init_UpdateHealth_Request_damage(::swarmz_interfaces::srv::UpdateHealth_Request & msg)
  : msg_(msg)
  {}
  ::swarmz_interfaces::srv::UpdateHealth_Request damage(::swarmz_interfaces::srv::UpdateHealth_Request::_damage_type arg)
  {
    msg_.damage = std::move(arg);
    return std::move(msg_);
  }

private:
  ::swarmz_interfaces::srv::UpdateHealth_Request msg_;
};

class Init_UpdateHealth_Request_robot_name
{
public:
  Init_UpdateHealth_Request_robot_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UpdateHealth_Request_damage robot_name(::swarmz_interfaces::srv::UpdateHealth_Request::_robot_name_type arg)
  {
    msg_.robot_name = std::move(arg);
    return Init_UpdateHealth_Request_damage(msg_);
  }

private:
  ::swarmz_interfaces::srv::UpdateHealth_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::swarmz_interfaces::srv::UpdateHealth_Request>()
{
  return swarmz_interfaces::srv::builder::Init_UpdateHealth_Request_robot_name();
}

}  // namespace swarmz_interfaces


namespace swarmz_interfaces
{

namespace srv
{

namespace builder
{

class Init_UpdateHealth_Response_success
{
public:
  Init_UpdateHealth_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::swarmz_interfaces::srv::UpdateHealth_Response success(::swarmz_interfaces::srv::UpdateHealth_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::swarmz_interfaces::srv::UpdateHealth_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::swarmz_interfaces::srv::UpdateHealth_Response>()
{
  return swarmz_interfaces::srv::builder::Init_UpdateHealth_Response_success();
}

}  // namespace swarmz_interfaces

#endif  // SWARMZ_INTERFACES__SRV__DETAIL__UPDATE_HEALTH__BUILDER_HPP_
