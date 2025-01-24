// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from swarmz_interfaces:srv/Missile.idl
// generated code does not contain a copyright notice

#ifndef SWARMZ_INTERFACES__SRV__DETAIL__MISSILE__STRUCT_HPP_
#define SWARMZ_INTERFACES__SRV__DETAIL__MISSILE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__swarmz_interfaces__srv__Missile_Request __attribute__((deprecated))
#else
# define DEPRECATED__swarmz_interfaces__srv__Missile_Request __declspec(deprecated)
#endif

namespace swarmz_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Missile_Request_
{
  using Type = Missile_Request_<ContainerAllocator>;

  explicit Missile_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_name = "";
    }
  }

  explicit Missile_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : robot_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_name = "";
    }
  }

  // field types and members
  using _robot_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _robot_name_type robot_name;

  // setters for named parameter idiom
  Type & set__robot_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->robot_name = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    swarmz_interfaces::srv::Missile_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const swarmz_interfaces::srv::Missile_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<swarmz_interfaces::srv::Missile_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<swarmz_interfaces::srv::Missile_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      swarmz_interfaces::srv::Missile_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<swarmz_interfaces::srv::Missile_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      swarmz_interfaces::srv::Missile_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<swarmz_interfaces::srv::Missile_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<swarmz_interfaces::srv::Missile_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<swarmz_interfaces::srv::Missile_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__swarmz_interfaces__srv__Missile_Request
    std::shared_ptr<swarmz_interfaces::srv::Missile_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__swarmz_interfaces__srv__Missile_Request
    std::shared_ptr<swarmz_interfaces::srv::Missile_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Missile_Request_ & other) const
  {
    if (this->robot_name != other.robot_name) {
      return false;
    }
    return true;
  }
  bool operator!=(const Missile_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Missile_Request_

// alias to use template instance with default allocator
using Missile_Request =
  swarmz_interfaces::srv::Missile_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace swarmz_interfaces


#ifndef _WIN32
# define DEPRECATED__swarmz_interfaces__srv__Missile_Response __attribute__((deprecated))
#else
# define DEPRECATED__swarmz_interfaces__srv__Missile_Response __declspec(deprecated)
#endif

namespace swarmz_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Missile_Response_
{
  using Type = Missile_Response_<ContainerAllocator>;

  explicit Missile_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->has_fired = false;
      this->ammo = 0;
    }
  }

  explicit Missile_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->has_fired = false;
      this->ammo = 0;
    }
  }

  // field types and members
  using _has_fired_type =
    bool;
  _has_fired_type has_fired;
  using _ammo_type =
    int8_t;
  _ammo_type ammo;

  // setters for named parameter idiom
  Type & set__has_fired(
    const bool & _arg)
  {
    this->has_fired = _arg;
    return *this;
  }
  Type & set__ammo(
    const int8_t & _arg)
  {
    this->ammo = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    swarmz_interfaces::srv::Missile_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const swarmz_interfaces::srv::Missile_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<swarmz_interfaces::srv::Missile_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<swarmz_interfaces::srv::Missile_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      swarmz_interfaces::srv::Missile_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<swarmz_interfaces::srv::Missile_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      swarmz_interfaces::srv::Missile_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<swarmz_interfaces::srv::Missile_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<swarmz_interfaces::srv::Missile_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<swarmz_interfaces::srv::Missile_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__swarmz_interfaces__srv__Missile_Response
    std::shared_ptr<swarmz_interfaces::srv::Missile_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__swarmz_interfaces__srv__Missile_Response
    std::shared_ptr<swarmz_interfaces::srv::Missile_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Missile_Response_ & other) const
  {
    if (this->has_fired != other.has_fired) {
      return false;
    }
    if (this->ammo != other.ammo) {
      return false;
    }
    return true;
  }
  bool operator!=(const Missile_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Missile_Response_

// alias to use template instance with default allocator
using Missile_Response =
  swarmz_interfaces::srv::Missile_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace swarmz_interfaces

namespace swarmz_interfaces
{

namespace srv
{

struct Missile
{
  using Request = swarmz_interfaces::srv::Missile_Request;
  using Response = swarmz_interfaces::srv::Missile_Response;
};

}  // namespace srv

}  // namespace swarmz_interfaces

#endif  // SWARMZ_INTERFACES__SRV__DETAIL__MISSILE__STRUCT_HPP_
