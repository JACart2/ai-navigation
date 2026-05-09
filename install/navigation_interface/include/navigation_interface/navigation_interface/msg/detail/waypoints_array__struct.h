// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from navigation_interface:msg/WaypointsArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "navigation_interface/msg/waypoints_array.h"


#ifndef NAVIGATION_INTERFACE__MSG__DETAIL__WAYPOINTS_ARRAY__STRUCT_H_
#define NAVIGATION_INTERFACE__MSG__DETAIL__WAYPOINTS_ARRAY__STRUCT_H_

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
// Member 'waypoints'
#include "sensor_msgs/msg/detail/nav_sat_fix__struct.h"

/// Struct defined in msg/WaypointsArray in the package navigation_interface.
typedef struct navigation_interface__msg__WaypointsArray
{
  std_msgs__msg__Header header;
  sensor_msgs__msg__NavSatFix__Sequence waypoints;
} navigation_interface__msg__WaypointsArray;

// Struct for a sequence of navigation_interface__msg__WaypointsArray.
typedef struct navigation_interface__msg__WaypointsArray__Sequence
{
  navigation_interface__msg__WaypointsArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} navigation_interface__msg__WaypointsArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // NAVIGATION_INTERFACE__MSG__DETAIL__WAYPOINTS_ARRAY__STRUCT_H_
