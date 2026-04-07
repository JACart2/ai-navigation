// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from motor_control_interface:msg/VelAngle.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "motor_control_interface/msg/vel_angle.hpp"


#ifndef MOTOR_CONTROL_INTERFACE__MSG__DETAIL__VEL_ANGLE__BUILDER_HPP_
#define MOTOR_CONTROL_INTERFACE__MSG__DETAIL__VEL_ANGLE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "motor_control_interface/msg/detail/vel_angle__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace motor_control_interface
{

namespace msg
{

namespace builder
{

class Init_VelAngle_angle
{
public:
  explicit Init_VelAngle_angle(::motor_control_interface::msg::VelAngle & msg)
  : msg_(msg)
  {}
  ::motor_control_interface::msg::VelAngle angle(::motor_control_interface::msg::VelAngle::_angle_type arg)
  {
    msg_.angle = std::move(arg);
    return std::move(msg_);
  }

private:
  ::motor_control_interface::msg::VelAngle msg_;
};

class Init_VelAngle_vel
{
public:
  explicit Init_VelAngle_vel(::motor_control_interface::msg::VelAngle & msg)
  : msg_(msg)
  {}
  Init_VelAngle_angle vel(::motor_control_interface::msg::VelAngle::_vel_type arg)
  {
    msg_.vel = std::move(arg);
    return Init_VelAngle_angle(msg_);
  }

private:
  ::motor_control_interface::msg::VelAngle msg_;
};

class Init_VelAngle_header
{
public:
  Init_VelAngle_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VelAngle_vel header(::motor_control_interface::msg::VelAngle::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_VelAngle_vel(msg_);
  }

private:
  ::motor_control_interface::msg::VelAngle msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::motor_control_interface::msg::VelAngle>()
{
  return motor_control_interface::msg::builder::Init_VelAngle_header();
}

}  // namespace motor_control_interface

#endif  // MOTOR_CONTROL_INTERFACE__MSG__DETAIL__VEL_ANGLE__BUILDER_HPP_
