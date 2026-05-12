// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from navigation_interface:msg/LatLongPoint.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "navigation_interface/msg/lat_long_point.hpp"


#ifndef NAVIGATION_INTERFACE__MSG__DETAIL__LAT_LONG_POINT__STRUCT_HPP_
#define NAVIGATION_INTERFACE__MSG__DETAIL__LAT_LONG_POINT__STRUCT_HPP_

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

#ifndef _WIN32
# define DEPRECATED__navigation_interface__msg__LatLongPoint __attribute__((deprecated))
#else
# define DEPRECATED__navigation_interface__msg__LatLongPoint __declspec(deprecated)
#endif

namespace navigation_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LatLongPoint_
{
  using Type = LatLongPoint_<ContainerAllocator>;

  explicit LatLongPoint_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->latitude = 0.0;
      this->longitude = 0.0;
      this->elevation = 0.0;
    }
  }

  explicit LatLongPoint_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->latitude = 0.0;
      this->longitude = 0.0;
      this->elevation = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _latitude_type =
    double;
  _latitude_type latitude;
  using _longitude_type =
    double;
  _longitude_type longitude;
  using _elevation_type =
    double;
  _elevation_type elevation;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__latitude(
    const double & _arg)
  {
    this->latitude = _arg;
    return *this;
  }
  Type & set__longitude(
    const double & _arg)
  {
    this->longitude = _arg;
    return *this;
  }
  Type & set__elevation(
    const double & _arg)
  {
    this->elevation = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    navigation_interface::msg::LatLongPoint_<ContainerAllocator> *;
  using ConstRawPtr =
    const navigation_interface::msg::LatLongPoint_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<navigation_interface::msg::LatLongPoint_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<navigation_interface::msg::LatLongPoint_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      navigation_interface::msg::LatLongPoint_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<navigation_interface::msg::LatLongPoint_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      navigation_interface::msg::LatLongPoint_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<navigation_interface::msg::LatLongPoint_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<navigation_interface::msg::LatLongPoint_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<navigation_interface::msg::LatLongPoint_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__navigation_interface__msg__LatLongPoint
    std::shared_ptr<navigation_interface::msg::LatLongPoint_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__navigation_interface__msg__LatLongPoint
    std::shared_ptr<navigation_interface::msg::LatLongPoint_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LatLongPoint_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->latitude != other.latitude) {
      return false;
    }
    if (this->longitude != other.longitude) {
      return false;
    }
    if (this->elevation != other.elevation) {
      return false;
    }
    return true;
  }
  bool operator!=(const LatLongPoint_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LatLongPoint_

// alias to use template instance with default allocator
using LatLongPoint =
  navigation_interface::msg::LatLongPoint_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace navigation_interface

#endif  // NAVIGATION_INTERFACE__MSG__DETAIL__LAT_LONG_POINT__STRUCT_HPP_
