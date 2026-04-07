// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from motor_control_interface:msg/VelAngle.idl
// generated code does not contain a copyright notice
#ifndef MOTOR_CONTROL_INTERFACE__MSG__DETAIL__VEL_ANGLE__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define MOTOR_CONTROL_INTERFACE__MSG__DETAIL__VEL_ANGLE__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "motor_control_interface/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "motor_control_interface/msg/detail/vel_angle__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_motor_control_interface
bool cdr_serialize_motor_control_interface__msg__VelAngle(
  const motor_control_interface__msg__VelAngle * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_motor_control_interface
bool cdr_deserialize_motor_control_interface__msg__VelAngle(
  eprosima::fastcdr::Cdr &,
  motor_control_interface__msg__VelAngle * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_motor_control_interface
size_t get_serialized_size_motor_control_interface__msg__VelAngle(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_motor_control_interface
size_t max_serialized_size_motor_control_interface__msg__VelAngle(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_motor_control_interface
bool cdr_serialize_key_motor_control_interface__msg__VelAngle(
  const motor_control_interface__msg__VelAngle * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_motor_control_interface
size_t get_serialized_size_key_motor_control_interface__msg__VelAngle(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_motor_control_interface
size_t max_serialized_size_key_motor_control_interface__msg__VelAngle(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_motor_control_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, motor_control_interface, msg, VelAngle)();

#ifdef __cplusplus
}
#endif

#endif  // MOTOR_CONTROL_INTERFACE__MSG__DETAIL__VEL_ANGLE__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
