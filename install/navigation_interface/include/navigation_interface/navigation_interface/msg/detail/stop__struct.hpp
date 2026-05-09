// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from navigation_interface:msg/Stop.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "navigation_interface/msg/stop.hpp"


#ifndef NAVIGATION_INTERFACE__MSG__DETAIL__STOP__STRUCT_HPP_
#define NAVIGATION_INTERFACE__MSG__DETAIL__STOP__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'sender_id'
#include "std_msgs/msg/detail/string__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__navigation_interface__msg__Stop __attribute__((deprecated))
#else
# define DEPRECATED__navigation_interface__msg__Stop __declspec(deprecated)
#endif

namespace navigation_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Stop_
{
  using Type = Stop_<ContainerAllocator>;

  explicit Stop_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    sender_id(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->stop = false;
      this->distance = 0.0;
    }
  }

  explicit Stop_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    sender_id(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->stop = false;
      this->distance = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _stop_type =
    bool;
  _stop_type stop;
  using _sender_id_type =
    std_msgs::msg::String_<ContainerAllocator>;
  _sender_id_type sender_id;
  using _distance_type =
    double;
  _distance_type distance;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__stop(
    const bool & _arg)
  {
    this->stop = _arg;
    return *this;
  }
  Type & set__sender_id(
    const std_msgs::msg::String_<ContainerAllocator> & _arg)
  {
    this->sender_id = _arg;
    return *this;
  }
  Type & set__distance(
    const double & _arg)
  {
    this->distance = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    navigation_interface::msg::Stop_<ContainerAllocator> *;
  using ConstRawPtr =
    const navigation_interface::msg::Stop_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<navigation_interface::msg::Stop_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<navigation_interface::msg::Stop_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      navigation_interface::msg::Stop_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<navigation_interface::msg::Stop_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      navigation_interface::msg::Stop_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<navigation_interface::msg::Stop_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<navigation_interface::msg::Stop_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<navigation_interface::msg::Stop_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__navigation_interface__msg__Stop
    std::shared_ptr<navigation_interface::msg::Stop_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__navigation_interface__msg__Stop
    std::shared_ptr<navigation_interface::msg::Stop_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Stop_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->stop != other.stop) {
      return false;
    }
    if (this->sender_id != other.sender_id) {
      return false;
    }
    if (this->distance != other.distance) {
      return false;
    }
    return true;
  }
  bool operator!=(const Stop_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Stop_

// alias to use template instance with default allocator
using Stop =
  navigation_interface::msg::Stop_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace navigation_interface

#endif  // NAVIGATION_INTERFACE__MSG__DETAIL__STOP__STRUCT_HPP_
