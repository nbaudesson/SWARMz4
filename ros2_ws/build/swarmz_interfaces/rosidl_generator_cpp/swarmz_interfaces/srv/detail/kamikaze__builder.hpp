// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from swarmz_interfaces:srv/Kamikaze.idl
// generated code does not contain a copyright notice

#ifndef SWARMZ_INTERFACES__SRV__DETAIL__KAMIKAZE__BUILDER_HPP_
#define SWARMZ_INTERFACES__SRV__DETAIL__KAMIKAZE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "swarmz_interfaces/srv/detail/kamikaze__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace swarmz_interfaces
{

namespace srv
{

namespace builder
{

class Init_Kamikaze_Request_robot_name
{
public:
  Init_Kamikaze_Request_robot_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::swarmz_interfaces::srv::Kamikaze_Request robot_name(::swarmz_interfaces::srv::Kamikaze_Request::_robot_name_type arg)
  {
    msg_.robot_name = std::move(arg);
    return std::move(msg_);
  }

private:
  ::swarmz_interfaces::srv::Kamikaze_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::swarmz_interfaces::srv::Kamikaze_Request>()
{
  return swarmz_interfaces::srv::builder::Init_Kamikaze_Request_robot_name();
}

}  // namespace swarmz_interfaces


namespace swarmz_interfaces
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::swarmz_interfaces::srv::Kamikaze_Response>()
{
  return ::swarmz_interfaces::srv::Kamikaze_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace swarmz_interfaces

#endif  // SWARMZ_INTERFACES__SRV__DETAIL__KAMIKAZE__BUILDER_HPP_
