// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from swarmz_interfaces:srv/Kamikaze.idl
// generated code does not contain a copyright notice

#ifndef SWARMZ_INTERFACES__SRV__DETAIL__KAMIKAZE__STRUCT_HPP_
#define SWARMZ_INTERFACES__SRV__DETAIL__KAMIKAZE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__swarmz_interfaces__srv__Kamikaze_Request __attribute__((deprecated))
#else
# define DEPRECATED__swarmz_interfaces__srv__Kamikaze_Request __declspec(deprecated)
#endif

namespace swarmz_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Kamikaze_Request_
{
  using Type = Kamikaze_Request_<ContainerAllocator>;

  explicit Kamikaze_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_name = "";
    }
  }

  explicit Kamikaze_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    swarmz_interfaces::srv::Kamikaze_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const swarmz_interfaces::srv::Kamikaze_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<swarmz_interfaces::srv::Kamikaze_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<swarmz_interfaces::srv::Kamikaze_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      swarmz_interfaces::srv::Kamikaze_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<swarmz_interfaces::srv::Kamikaze_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      swarmz_interfaces::srv::Kamikaze_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<swarmz_interfaces::srv::Kamikaze_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<swarmz_interfaces::srv::Kamikaze_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<swarmz_interfaces::srv::Kamikaze_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__swarmz_interfaces__srv__Kamikaze_Request
    std::shared_ptr<swarmz_interfaces::srv::Kamikaze_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__swarmz_interfaces__srv__Kamikaze_Request
    std::shared_ptr<swarmz_interfaces::srv::Kamikaze_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Kamikaze_Request_ & other) const
  {
    if (this->robot_name != other.robot_name) {
      return false;
    }
    return true;
  }
  bool operator!=(const Kamikaze_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Kamikaze_Request_

// alias to use template instance with default allocator
using Kamikaze_Request =
  swarmz_interfaces::srv::Kamikaze_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace swarmz_interfaces


#ifndef _WIN32
# define DEPRECATED__swarmz_interfaces__srv__Kamikaze_Response __attribute__((deprecated))
#else
# define DEPRECATED__swarmz_interfaces__srv__Kamikaze_Response __declspec(deprecated)
#endif

namespace swarmz_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Kamikaze_Response_
{
  using Type = Kamikaze_Response_<ContainerAllocator>;

  explicit Kamikaze_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit Kamikaze_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    swarmz_interfaces::srv::Kamikaze_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const swarmz_interfaces::srv::Kamikaze_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<swarmz_interfaces::srv::Kamikaze_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<swarmz_interfaces::srv::Kamikaze_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      swarmz_interfaces::srv::Kamikaze_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<swarmz_interfaces::srv::Kamikaze_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      swarmz_interfaces::srv::Kamikaze_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<swarmz_interfaces::srv::Kamikaze_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<swarmz_interfaces::srv::Kamikaze_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<swarmz_interfaces::srv::Kamikaze_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__swarmz_interfaces__srv__Kamikaze_Response
    std::shared_ptr<swarmz_interfaces::srv::Kamikaze_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__swarmz_interfaces__srv__Kamikaze_Response
    std::shared_ptr<swarmz_interfaces::srv::Kamikaze_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Kamikaze_Response_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const Kamikaze_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Kamikaze_Response_

// alias to use template instance with default allocator
using Kamikaze_Response =
  swarmz_interfaces::srv::Kamikaze_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace swarmz_interfaces

namespace swarmz_interfaces
{

namespace srv
{

struct Kamikaze
{
  using Request = swarmz_interfaces::srv::Kamikaze_Request;
  using Response = swarmz_interfaces::srv::Kamikaze_Response;
};

}  // namespace srv

}  // namespace swarmz_interfaces

#endif  // SWARMZ_INTERFACES__SRV__DETAIL__KAMIKAZE__STRUCT_HPP_
