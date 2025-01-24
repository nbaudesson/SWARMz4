// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from swarmz_interfaces:msg/Detections.idl
// generated code does not contain a copyright notice

#ifndef SWARMZ_INTERFACES__MSG__DETAIL__DETECTIONS__BUILDER_HPP_
#define SWARMZ_INTERFACES__MSG__DETAIL__DETECTIONS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "swarmz_interfaces/msg/detail/detections__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace swarmz_interfaces
{

namespace msg
{

namespace builder
{

class Init_Detections_detections
{
public:
  explicit Init_Detections_detections(::swarmz_interfaces::msg::Detections & msg)
  : msg_(msg)
  {}
  ::swarmz_interfaces::msg::Detections detections(::swarmz_interfaces::msg::Detections::_detections_type arg)
  {
    msg_.detections = std::move(arg);
    return std::move(msg_);
  }

private:
  ::swarmz_interfaces::msg::Detections msg_;
};

class Init_Detections_header
{
public:
  Init_Detections_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Detections_detections header(::swarmz_interfaces::msg::Detections::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Detections_detections(msg_);
  }

private:
  ::swarmz_interfaces::msg::Detections msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::swarmz_interfaces::msg::Detections>()
{
  return swarmz_interfaces::msg::builder::Init_Detections_header();
}

}  // namespace swarmz_interfaces

#endif  // SWARMZ_INTERFACES__MSG__DETAIL__DETECTIONS__BUILDER_HPP_
