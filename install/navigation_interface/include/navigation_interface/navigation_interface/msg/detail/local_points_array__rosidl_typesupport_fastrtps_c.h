// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from navigation_interface:msg/LocalPointsArray.idl
// generated code does not contain a copyright notice
#ifndef NAVIGATION_INTERFACE__MSG__DETAIL__LOCAL_POINTS_ARRAY__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define NAVIGATION_INTERFACE__MSG__DETAIL__LOCAL_POINTS_ARRAY__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "navigation_interface/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "navigation_interface/msg/detail/local_points_array__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_navigation_interface
bool cdr_serialize_navigation_interface__msg__LocalPointsArray(
  const navigation_interface__msg__LocalPointsArray * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_navigation_interface
bool cdr_deserialize_navigation_interface__msg__LocalPointsArray(
  eprosima::fastcdr::Cdr &,
  navigation_interface__msg__LocalPointsArray * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_navigation_interface
size_t get_serialized_size_navigation_interface__msg__LocalPointsArray(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_navigation_interface
size_t max_serialized_size_navigation_interface__msg__LocalPointsArray(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_navigation_interface
bool cdr_serialize_key_navigation_interface__msg__LocalPointsArray(
  const navigation_interface__msg__LocalPointsArray * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_navigation_interface
size_t get_serialized_size_key_navigation_interface__msg__LocalPointsArray(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_navigation_interface
size_t max_serialized_size_key_navigation_interface__msg__LocalPointsArray(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_navigation_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, navigation_interface, msg, LocalPointsArray)();

#ifdef __cplusplus
}
#endif

#endif  // NAVIGATION_INTERFACE__MSG__DETAIL__LOCAL_POINTS_ARRAY__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
