// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from navigation_interface:msg/VehicleState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "navigation_interface/msg/vehicle_state.hpp"


#ifndef NAVIGATION_INTERFACE__MSG__DETAIL__VEHICLE_STATE__TRAITS_HPP_
#define NAVIGATION_INTERFACE__MSG__DETAIL__VEHICLE_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "navigation_interface/msg/detail/vehicle_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace navigation_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const VehicleState & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: is_navigating
  {
    out << "is_navigating: ";
    rosidl_generator_traits::value_to_yaml(msg.is_navigating, out);
    out << ", ";
  }

  // member: reached_destination
  {
    out << "reached_destination: ";
    rosidl_generator_traits::value_to_yaml(msg.reached_destination, out);
    out << ", ";
  }

  // member: stopped
  {
    out << "stopped: ";
    rosidl_generator_traits::value_to_yaml(msg.stopped, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const VehicleState & msg,
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

  // member: is_navigating
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_navigating: ";
    rosidl_generator_traits::value_to_yaml(msg.is_navigating, out);
    out << "\n";
  }

  // member: reached_destination
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reached_destination: ";
    rosidl_generator_traits::value_to_yaml(msg.reached_destination, out);
    out << "\n";
  }

  // member: stopped
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stopped: ";
    rosidl_generator_traits::value_to_yaml(msg.stopped, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const VehicleState & msg, bool use_flow_style = false)
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

}  // namespace navigation_interface

namespace rosidl_generator_traits
{

[[deprecated("use navigation_interface::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const navigation_interface::msg::VehicleState & msg,
  std::ostream & out, size_t indentation = 0)
{
  navigation_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use navigation_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const navigation_interface::msg::VehicleState & msg)
{
  return navigation_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<navigation_interface::msg::VehicleState>()
{
  return "navigation_interface::msg::VehicleState";
}

template<>
inline const char * name<navigation_interface::msg::VehicleState>()
{
  return "navigation_interface/msg/VehicleState";
}

template<>
struct has_fixed_size<navigation_interface::msg::VehicleState>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<navigation_interface::msg::VehicleState>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<navigation_interface::msg::VehicleState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // NAVIGATION_INTERFACE__MSG__DETAIL__VEHICLE_STATE__TRAITS_HPP_
