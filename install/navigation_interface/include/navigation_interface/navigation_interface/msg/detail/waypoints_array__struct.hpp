// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from navigation_interface:msg/WaypointsArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "navigation_interface/msg/waypoints_array.hpp"


#ifndef NAVIGATION_INTERFACE__MSG__DETAIL__WAYPOINTS_ARRAY__STRUCT_HPP_
#define NAVIGATION_INTERFACE__MSG__DETAIL__WAYPOINTS_ARRAY__STRUCT_HPP_

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
// Member 'waypoints'
#include "sensor_msgs/msg/detail/nav_sat_fix__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__navigation_interface__msg__WaypointsArray __attribute__((deprecated))
#else
# define DEPRECATED__navigation_interface__msg__WaypointsArray __declspec(deprecated)
#endif

namespace navigation_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct WaypointsArray_
{
  using Type = WaypointsArray_<ContainerAllocator>;

  explicit WaypointsArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit WaypointsArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _waypoints_type =
    std::vector<sensor_msgs::msg::NavSatFix_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<sensor_msgs::msg::NavSatFix_<ContainerAllocator>>>;
  _waypoints_type waypoints;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__waypoints(
    const std::vector<sensor_msgs::msg::NavSatFix_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<sensor_msgs::msg::NavSatFix_<ContainerAllocator>>> & _arg)
  {
    this->waypoints = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    navigation_interface::msg::WaypointsArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const navigation_interface::msg::WaypointsArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<navigation_interface::msg::WaypointsArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<navigation_interface::msg::WaypointsArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      navigation_interface::msg::WaypointsArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<navigation_interface::msg::WaypointsArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      navigation_interface::msg::WaypointsArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<navigation_interface::msg::WaypointsArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<navigation_interface::msg::WaypointsArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<navigation_interface::msg::WaypointsArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__navigation_interface__msg__WaypointsArray
    std::shared_ptr<navigation_interface::msg::WaypointsArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__navigation_interface__msg__WaypointsArray
    std::shared_ptr<navigation_interface::msg::WaypointsArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const WaypointsArray_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->waypoints != other.waypoints) {
      return false;
    }
    return true;
  }
  bool operator!=(const WaypointsArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct WaypointsArray_

// alias to use template instance with default allocator
using WaypointsArray =
  navigation_interface::msg::WaypointsArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace navigation_interface

#endif  // NAVIGATION_INTERFACE__MSG__DETAIL__WAYPOINTS_ARRAY__STRUCT_HPP_
