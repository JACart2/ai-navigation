// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from navigation_interface:msg/LatLongPoint.idl
// generated code does not contain a copyright notice

#ifndef NAVIGATION_INTERFACE__MSG__DETAIL__LAT_LONG_POINT__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define NAVIGATION_INTERFACE__MSG__DETAIL__LAT_LONG_POINT__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include <cstddef>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "navigation_interface/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "navigation_interface/msg/detail/lat_long_point__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace navigation_interface
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_navigation_interface
cdr_serialize(
  const navigation_interface::msg::LatLongPoint & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_navigation_interface
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  navigation_interface::msg::LatLongPoint & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_navigation_interface
get_serialized_size(
  const navigation_interface::msg::LatLongPoint & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_navigation_interface
max_serialized_size_LatLongPoint(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_navigation_interface
cdr_serialize_key(
  const navigation_interface::msg::LatLongPoint & ros_message,
  eprosima::fastcdr::Cdr &);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_navigation_interface
get_serialized_size_key(
  const navigation_interface::msg::LatLongPoint & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_navigation_interface
max_serialized_size_key_LatLongPoint(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace navigation_interface

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_navigation_interface
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, navigation_interface, msg, LatLongPoint)();

#ifdef __cplusplus
}
#endif

#endif  // NAVIGATION_INTERFACE__MSG__DETAIL__LAT_LONG_POINT__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
