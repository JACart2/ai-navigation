// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from navigation_interface:msg/ObstacleArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "navigation_interface/msg/obstacle_array.hpp"


#ifndef NAVIGATION_INTERFACE__MSG__DETAIL__OBSTACLE_ARRAY__TRAITS_HPP_
#define NAVIGATION_INTERFACE__MSG__DETAIL__OBSTACLE_ARRAY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "navigation_interface/msg/detail/obstacle_array__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'obstacles'
#include "navigation_interface/msg/detail/obstacle__traits.hpp"

namespace navigation_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const ObstacleArray & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: obstacles
  {
    if (msg.obstacles.size() == 0) {
      out << "obstacles: []";
    } else {
      out << "obstacles: [";
      size_t pending_items = msg.obstacles.size();
      for (auto item : msg.obstacles) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ObstacleArray & msg,
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

  // member: obstacles
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.obstacles.size() == 0) {
      out << "obstacles: []\n";
    } else {
      out << "obstacles:\n";
      for (auto item : msg.obstacles) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ObstacleArray & msg, bool use_flow_style = false)
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
  const navigation_interface::msg::ObstacleArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  navigation_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use navigation_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const navigation_interface::msg::ObstacleArray & msg)
{
  return navigation_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<navigation_interface::msg::ObstacleArray>()
{
  return "navigation_interface::msg::ObstacleArray";
}

template<>
inline const char * name<navigation_interface::msg::ObstacleArray>()
{
  return "navigation_interface/msg/ObstacleArray";
}

template<>
struct has_fixed_size<navigation_interface::msg::ObstacleArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<navigation_interface::msg::ObstacleArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<navigation_interface::msg::ObstacleArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // NAVIGATION_INTERFACE__MSG__DETAIL__OBSTACLE_ARRAY__TRAITS_HPP_
