// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from navigation_interface:msg/LatLongPoint.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "navigation_interface/msg/lat_long_point.h"


#ifndef NAVIGATION_INTERFACE__MSG__DETAIL__LAT_LONG_POINT__STRUCT_H_
#define NAVIGATION_INTERFACE__MSG__DETAIL__LAT_LONG_POINT__STRUCT_H_

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

/// Struct defined in msg/LatLongPoint in the package navigation_interface.
typedef struct navigation_interface__msg__LatLongPoint
{
  std_msgs__msg__Header header;
  double latitude;
  double longitude;
  double elevation;
} navigation_interface__msg__LatLongPoint;

// Struct for a sequence of navigation_interface__msg__LatLongPoint.
typedef struct navigation_interface__msg__LatLongPoint__Sequence
{
  navigation_interface__msg__LatLongPoint * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} navigation_interface__msg__LatLongPoint__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // NAVIGATION_INTERFACE__MSG__DETAIL__LAT_LONG_POINT__STRUCT_H_
