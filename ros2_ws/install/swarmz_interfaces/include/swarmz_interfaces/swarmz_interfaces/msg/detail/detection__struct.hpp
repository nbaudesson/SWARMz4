// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from swarmz_interfaces:msg/Detection.idl
// generated code does not contain a copyright notice

#ifndef SWARMZ_INTERFACES__MSG__DETAIL__DETECTION__STRUCT_HPP_
#define SWARMZ_INTERFACES__MSG__DETAIL__DETECTION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'relative_position'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__swarmz_interfaces__msg__Detection __attribute__((deprecated))
#else
# define DEPRECATED__swarmz_interfaces__msg__Detection __declspec(deprecated)
#endif

namespace swarmz_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Detection_
{
  using Type = Detection_<ContainerAllocator>;

  explicit Detection_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : relative_position(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->vehicle_type = 0;
      this->is_friend = false;
    }
  }

  explicit Detection_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : relative_position(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->vehicle_type = 0;
      this->is_friend = false;
    }
  }

  // field types and members
  using _vehicle_type_type =
    int8_t;
  _vehicle_type_type vehicle_type;
  using _is_friend_type =
    bool;
  _is_friend_type is_friend;
  using _relative_position_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _relative_position_type relative_position;

  // setters for named parameter idiom
  Type & set__vehicle_type(
    const int8_t & _arg)
  {
    this->vehicle_type = _arg;
    return *this;
  }
  Type & set__is_friend(
    const bool & _arg)
  {
    this->is_friend = _arg;
    return *this;
  }
  Type & set__relative_position(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->relative_position = _arg;
    return *this;
  }

  // constant declarations
  static constexpr int8_t DRONE =
    0;
  static constexpr int8_t SHIP =
    1;

  // pointer types
  using RawPtr =
    swarmz_interfaces::msg::Detection_<ContainerAllocator> *;
  using ConstRawPtr =
    const swarmz_interfaces::msg::Detection_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<swarmz_interfaces::msg::Detection_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<swarmz_interfaces::msg::Detection_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      swarmz_interfaces::msg::Detection_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<swarmz_interfaces::msg::Detection_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      swarmz_interfaces::msg::Detection_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<swarmz_interfaces::msg::Detection_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<swarmz_interfaces::msg::Detection_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<swarmz_interfaces::msg::Detection_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__swarmz_interfaces__msg__Detection
    std::shared_ptr<swarmz_interfaces::msg::Detection_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__swarmz_interfaces__msg__Detection
    std::shared_ptr<swarmz_interfaces::msg::Detection_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Detection_ & other) const
  {
    if (this->vehicle_type != other.vehicle_type) {
      return false;
    }
    if (this->is_friend != other.is_friend) {
      return false;
    }
    if (this->relative_position != other.relative_position) {
      return false;
    }
    return true;
  }
  bool operator!=(const Detection_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Detection_

// alias to use template instance with default allocator
using Detection =
  swarmz_interfaces::msg::Detection_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int8_t Detection_<ContainerAllocator>::DRONE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int8_t Detection_<ContainerAllocator>::SHIP;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace swarmz_interfaces

#endif  // SWARMZ_INTERFACES__MSG__DETAIL__DETECTION__STRUCT_HPP_
