// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from swarmz_interfaces:srv/UpdateHealth.idl
// generated code does not contain a copyright notice

#ifndef SWARMZ_INTERFACES__SRV__DETAIL__UPDATE_HEALTH__STRUCT_HPP_
#define SWARMZ_INTERFACES__SRV__DETAIL__UPDATE_HEALTH__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__swarmz_interfaces__srv__UpdateHealth_Request __attribute__((deprecated))
#else
# define DEPRECATED__swarmz_interfaces__srv__UpdateHealth_Request __declspec(deprecated)
#endif

namespace swarmz_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct UpdateHealth_Request_
{
  using Type = UpdateHealth_Request_<ContainerAllocator>;

  explicit UpdateHealth_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_name = "";
      this->damage = 0l;
    }
  }

  explicit UpdateHealth_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : robot_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_name = "";
      this->damage = 0l;
    }
  }

  // field types and members
  using _robot_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _robot_name_type robot_name;
  using _damage_type =
    int32_t;
  _damage_type damage;

  // setters for named parameter idiom
  Type & set__robot_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->robot_name = _arg;
    return *this;
  }
  Type & set__damage(
    const int32_t & _arg)
  {
    this->damage = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    swarmz_interfaces::srv::UpdateHealth_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const swarmz_interfaces::srv::UpdateHealth_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<swarmz_interfaces::srv::UpdateHealth_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<swarmz_interfaces::srv::UpdateHealth_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      swarmz_interfaces::srv::UpdateHealth_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<swarmz_interfaces::srv::UpdateHealth_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      swarmz_interfaces::srv::UpdateHealth_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<swarmz_interfaces::srv::UpdateHealth_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<swarmz_interfaces::srv::UpdateHealth_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<swarmz_interfaces::srv::UpdateHealth_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__swarmz_interfaces__srv__UpdateHealth_Request
    std::shared_ptr<swarmz_interfaces::srv::UpdateHealth_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__swarmz_interfaces__srv__UpdateHealth_Request
    std::shared_ptr<swarmz_interfaces::srv::UpdateHealth_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UpdateHealth_Request_ & other) const
  {
    if (this->robot_name != other.robot_name) {
      return false;
    }
    if (this->damage != other.damage) {
      return false;
    }
    return true;
  }
  bool operator!=(const UpdateHealth_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UpdateHealth_Request_

// alias to use template instance with default allocator
using UpdateHealth_Request =
  swarmz_interfaces::srv::UpdateHealth_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace swarmz_interfaces


#ifndef _WIN32
# define DEPRECATED__swarmz_interfaces__srv__UpdateHealth_Response __attribute__((deprecated))
#else
# define DEPRECATED__swarmz_interfaces__srv__UpdateHealth_Response __declspec(deprecated)
#endif

namespace swarmz_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct UpdateHealth_Response_
{
  using Type = UpdateHealth_Response_<ContainerAllocator>;

  explicit UpdateHealth_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit UpdateHealth_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    swarmz_interfaces::srv::UpdateHealth_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const swarmz_interfaces::srv::UpdateHealth_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<swarmz_interfaces::srv::UpdateHealth_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<swarmz_interfaces::srv::UpdateHealth_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      swarmz_interfaces::srv::UpdateHealth_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<swarmz_interfaces::srv::UpdateHealth_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      swarmz_interfaces::srv::UpdateHealth_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<swarmz_interfaces::srv::UpdateHealth_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<swarmz_interfaces::srv::UpdateHealth_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<swarmz_interfaces::srv::UpdateHealth_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__swarmz_interfaces__srv__UpdateHealth_Response
    std::shared_ptr<swarmz_interfaces::srv::UpdateHealth_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__swarmz_interfaces__srv__UpdateHealth_Response
    std::shared_ptr<swarmz_interfaces::srv::UpdateHealth_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UpdateHealth_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const UpdateHealth_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UpdateHealth_Response_

// alias to use template instance with default allocator
using UpdateHealth_Response =
  swarmz_interfaces::srv::UpdateHealth_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace swarmz_interfaces

namespace swarmz_interfaces
{

namespace srv
{

struct UpdateHealth
{
  using Request = swarmz_interfaces::srv::UpdateHealth_Request;
  using Response = swarmz_interfaces::srv::UpdateHealth_Response;
};

}  // namespace srv

}  // namespace swarmz_interfaces

#endif  // SWARMZ_INTERFACES__SRV__DETAIL__UPDATE_HEALTH__STRUCT_HPP_
