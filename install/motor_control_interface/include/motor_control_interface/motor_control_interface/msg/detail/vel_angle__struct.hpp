// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from motor_control_interface:msg/VelAngle.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "motor_control_interface/msg/vel_angle.hpp"


#ifndef MOTOR_CONTROL_INTERFACE__MSG__DETAIL__VEL_ANGLE__STRUCT_HPP_
#define MOTOR_CONTROL_INTERFACE__MSG__DETAIL__VEL_ANGLE__STRUCT_HPP_

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
# define DEPRECATED__motor_control_interface__msg__VelAngle __attribute__((deprecated))
#else
# define DEPRECATED__motor_control_interface__msg__VelAngle __declspec(deprecated)
#endif

namespace motor_control_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct VelAngle_
{
  using Type = VelAngle_<ContainerAllocator>;

  explicit VelAngle_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->vel = 0.0;
      this->angle = 0.0;
    }
  }

  explicit VelAngle_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->vel = 0.0;
      this->angle = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _vel_type =
    double;
  _vel_type vel;
  using _angle_type =
    double;
  _angle_type angle;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__vel(
    const double & _arg)
  {
    this->vel = _arg;
    return *this;
  }
  Type & set__angle(
    const double & _arg)
  {
    this->angle = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    motor_control_interface::msg::VelAngle_<ContainerAllocator> *;
  using ConstRawPtr =
    const motor_control_interface::msg::VelAngle_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<motor_control_interface::msg::VelAngle_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<motor_control_interface::msg::VelAngle_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      motor_control_interface::msg::VelAngle_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<motor_control_interface::msg::VelAngle_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      motor_control_interface::msg::VelAngle_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<motor_control_interface::msg::VelAngle_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<motor_control_interface::msg::VelAngle_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<motor_control_interface::msg::VelAngle_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__motor_control_interface__msg__VelAngle
    std::shared_ptr<motor_control_interface::msg::VelAngle_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__motor_control_interface__msg__VelAngle
    std::shared_ptr<motor_control_interface::msg::VelAngle_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VelAngle_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->vel != other.vel) {
      return false;
    }
    if (this->angle != other.angle) {
      return false;
    }
    return true;
  }
  bool operator!=(const VelAngle_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VelAngle_

// alias to use template instance with default allocator
using VelAngle =
  motor_control_interface::msg::VelAngle_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace motor_control_interface

#endif  // MOTOR_CONTROL_INTERFACE__MSG__DETAIL__VEL_ANGLE__STRUCT_HPP_
