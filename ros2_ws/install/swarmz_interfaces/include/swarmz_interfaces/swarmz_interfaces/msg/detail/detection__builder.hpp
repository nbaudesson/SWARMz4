// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from swarmz_interfaces:msg/Detection.idl
// generated code does not contain a copyright notice

#ifndef SWARMZ_INTERFACES__MSG__DETAIL__DETECTION__BUILDER_HPP_
#define SWARMZ_INTERFACES__MSG__DETAIL__DETECTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "swarmz_interfaces/msg/detail/detection__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace swarmz_interfaces
{

namespace msg
{

namespace builder
{

class Init_Detection_relative_position
{
public:
  explicit Init_Detection_relative_position(::swarmz_interfaces::msg::Detection & msg)
  : msg_(msg)
  {}
  ::swarmz_interfaces::msg::Detection relative_position(::swarmz_interfaces::msg::Detection::_relative_position_type arg)
  {
    msg_.relative_position = std::move(arg);
    return std::move(msg_);
  }

private:
  ::swarmz_interfaces::msg::Detection msg_;
};

class Init_Detection_is_friend
{
public:
  explicit Init_Detection_is_friend(::swarmz_interfaces::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_relative_position is_friend(::swarmz_interfaces::msg::Detection::_is_friend_type arg)
  {
    msg_.is_friend = std::move(arg);
    return Init_Detection_relative_position(msg_);
  }

private:
  ::swarmz_interfaces::msg::Detection msg_;
};

class Init_Detection_vehicle_type
{
public:
  Init_Detection_vehicle_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Detection_is_friend vehicle_type(::swarmz_interfaces::msg::Detection::_vehicle_type_type arg)
  {
    msg_.vehicle_type = std::move(arg);
    return Init_Detection_is_friend(msg_);
  }

private:
  ::swarmz_interfaces::msg::Detection msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::swarmz_interfaces::msg::Detection>()
{
  return swarmz_interfaces::msg::builder::Init_Detection_vehicle_type();
}

}  // namespace swarmz_interfaces

#endif  // SWARMZ_INTERFACES__MSG__DETAIL__DETECTION__BUILDER_HPP_
