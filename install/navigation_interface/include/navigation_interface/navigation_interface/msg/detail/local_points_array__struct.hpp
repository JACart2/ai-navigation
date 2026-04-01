// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from navigation_interface:msg/LocalPointsArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "navigation_interface/msg/local_points_array.hpp"


#ifndef NAVIGATION_INTERFACE__MSG__DETAIL__LOCAL_POINTS_ARRAY__STRUCT_HPP_
#define NAVIGATION_INTERFACE__MSG__DETAIL__LOCAL_POINTS_ARRAY__STRUCT_HPP_

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
// Member 'localpoints'
#include "geometry_msgs/msg/detail/pose__struct.hpp"
// Member 'total_distance'
#include "std_msgs/msg/detail/float64__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__navigation_interface__msg__LocalPointsArray __attribute__((deprecated))
#else
# define DEPRECATED__navigation_interface__msg__LocalPointsArray __declspec(deprecated)
#endif

namespace navigation_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LocalPointsArray_
{
  using Type = LocalPointsArray_<ContainerAllocator>;

  explicit LocalPointsArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    total_distance(_init)
  {
    (void)_init;
  }

  explicit LocalPointsArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    total_distance(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _localpoints_type =
    std::vector<geometry_msgs::msg::Pose_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Pose_<ContainerAllocator>>>;
  _localpoints_type localpoints;
  using _total_distance_type =
    std_msgs::msg::Float64_<ContainerAllocator>;
  _total_distance_type total_distance;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__localpoints(
    const std::vector<geometry_msgs::msg::Pose_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Pose_<ContainerAllocator>>> & _arg)
  {
    this->localpoints = _arg;
    return *this;
  }
  Type & set__total_distance(
    const std_msgs::msg::Float64_<ContainerAllocator> & _arg)
  {
    this->total_distance = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    navigation_interface::msg::LocalPointsArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const navigation_interface::msg::LocalPointsArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<navigation_interface::msg::LocalPointsArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<navigation_interface::msg::LocalPointsArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      navigation_interface::msg::LocalPointsArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<navigation_interface::msg::LocalPointsArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      navigation_interface::msg::LocalPointsArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<navigation_interface::msg::LocalPointsArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<navigation_interface::msg::LocalPointsArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<navigation_interface::msg::LocalPointsArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__navigation_interface__msg__LocalPointsArray
    std::shared_ptr<navigation_interface::msg::LocalPointsArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__navigation_interface__msg__LocalPointsArray
    std::shared_ptr<navigation_interface::msg::LocalPointsArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LocalPointsArray_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->localpoints != other.localpoints) {
      return false;
    }
    if (this->total_distance != other.total_distance) {
      return false;
    }
    return true;
  }
  bool operator!=(const LocalPointsArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LocalPointsArray_

// alias to use template instance with default allocator
using LocalPointsArray =
  navigation_interface::msg::LocalPointsArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace navigation_interface

#endif  // NAVIGATION_INTERFACE__MSG__DETAIL__LOCAL_POINTS_ARRAY__STRUCT_HPP_
