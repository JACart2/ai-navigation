// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from navigation_interface:msg/LocalPointsArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "navigation_interface/msg/local_points_array.hpp"


#ifndef NAVIGATION_INTERFACE__MSG__DETAIL__LOCAL_POINTS_ARRAY__TRAITS_HPP_
#define NAVIGATION_INTERFACE__MSG__DETAIL__LOCAL_POINTS_ARRAY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "navigation_interface/msg/detail/local_points_array__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'localpoints'
#include "geometry_msgs/msg/detail/pose__traits.hpp"
// Member 'total_distance'
#include "std_msgs/msg/detail/float64__traits.hpp"

namespace navigation_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const LocalPointsArray & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: localpoints
  {
    if (msg.localpoints.size() == 0) {
      out << "localpoints: []";
    } else {
      out << "localpoints: [";
      size_t pending_items = msg.localpoints.size();
      for (auto item : msg.localpoints) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: total_distance
  {
    out << "total_distance: ";
    to_flow_style_yaml(msg.total_distance, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LocalPointsArray & msg,
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

  // member: localpoints
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.localpoints.size() == 0) {
      out << "localpoints: []\n";
    } else {
      out << "localpoints:\n";
      for (auto item : msg.localpoints) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: total_distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "total_distance:\n";
    to_block_style_yaml(msg.total_distance, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LocalPointsArray & msg, bool use_flow_style = false)
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
  const navigation_interface::msg::LocalPointsArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  navigation_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use navigation_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const navigation_interface::msg::LocalPointsArray & msg)
{
  return navigation_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<navigation_interface::msg::LocalPointsArray>()
{
  return "navigation_interface::msg::LocalPointsArray";
}

template<>
inline const char * name<navigation_interface::msg::LocalPointsArray>()
{
  return "navigation_interface/msg/LocalPointsArray";
}

template<>
struct has_fixed_size<navigation_interface::msg::LocalPointsArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<navigation_interface::msg::LocalPointsArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<navigation_interface::msg::LocalPointsArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // NAVIGATION_INTERFACE__MSG__DETAIL__LOCAL_POINTS_ARRAY__TRAITS_HPP_
