// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from navigation_interface:msg/VehicleState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "navigation_interface/msg/vehicle_state.h"


#ifndef NAVIGATION_INTERFACE__MSG__DETAIL__VEHICLE_STATE__STRUCT_H_
#define NAVIGATION_INTERFACE__MSG__DETAIL__VEHICLE_STATE__STRUCT_H_

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

/// Struct defined in msg/VehicleState in the package navigation_interface.
typedef struct navigation_interface__msg__VehicleState
{
  std_msgs__msg__Header header;
  bool is_navigating;
  bool reached_destination;
  bool stopped;
} navigation_interface__msg__VehicleState;

// Struct for a sequence of navigation_interface__msg__VehicleState.
typedef struct navigation_interface__msg__VehicleState__Sequence
{
  navigation_interface__msg__VehicleState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} navigation_interface__msg__VehicleState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // NAVIGATION_INTERFACE__MSG__DETAIL__VEHICLE_STATE__STRUCT_H_
