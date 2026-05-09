// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from motor_control_interface:msg/VelAngle.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "motor_control_interface/msg/vel_angle.h"


#ifndef MOTOR_CONTROL_INTERFACE__MSG__DETAIL__VEL_ANGLE__STRUCT_H_
#define MOTOR_CONTROL_INTERFACE__MSG__DETAIL__VEL_ANGLE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/VelAngle in the package motor_control_interface.
typedef struct motor_control_interface__msg__VelAngle
{
  std_msgs__msg__Header header;
  double vel;
  double angle;
} motor_control_interface__msg__VelAngle;

// Struct for a sequence of motor_control_interface__msg__VelAngle.
typedef struct motor_control_interface__msg__VelAngle__Sequence
{
  motor_control_interface__msg__VelAngle * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} motor_control_interface__msg__VelAngle__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MOTOR_CONTROL_INTERFACE__MSG__DETAIL__VEL_ANGLE__STRUCT_H_
