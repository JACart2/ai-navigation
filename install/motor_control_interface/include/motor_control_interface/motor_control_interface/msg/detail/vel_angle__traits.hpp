// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from motor_control_interface:msg/VelAngle.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "motor_control_interface/msg/vel_angle.hpp"


#ifndef MOTOR_CONTROL_INTERFACE__MSG__DETAIL__VEL_ANGLE__TRAITS_HPP_
#define MOTOR_CONTROL_INTERFACE__MSG__DETAIL__VEL_ANGLE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "motor_control_interface/msg/detail/vel_angle__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace motor_control_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const VelAngle & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: vel
  {
    out << "vel: ";
    rosidl_generator_traits::value_to_yaml(msg.vel, out);
    out << ", ";
  }

  // member: angle
  {
    out << "angle: ";
    rosidl_generator_traits::value_to_yaml(msg.angle, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const VelAngle & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: vel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vel: ";
    rosidl_generator_traits::value_to_yaml(msg.vel, out);
    out << "\n";
  }

  // member: angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angle: ";
    rosidl_generator_traits::value_to_yaml(msg.angle, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const VelAngle & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace motor_control_interface

namespace rosidl_generator_traits
{

[[deprecated("use motor_control_interface::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const motor_control_interface::msg::VelAngle & msg,
  std::ostream & out, size_t indentation = 0)
{
  motor_control_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use motor_control_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const motor_control_interface::msg::VelAngle & msg)
{
  return motor_control_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<motor_control_interface::msg::VelAngle>()
{
  return "motor_control_interface::msg::VelAngle";
}

template<>
inline const char * name<motor_control_interface::msg::VelAngle>()
{
  return "motor_control_interface/msg/VelAngle";
}

template<>
struct has_fixed_size<motor_control_interface::msg::VelAngle>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<motor_control_interface::msg::VelAngle>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<motor_control_interface::msg::VelAngle>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MOTOR_CONTROL_INTERFACE__MSG__DETAIL__VEL_ANGLE__TRAITS_HPP_
