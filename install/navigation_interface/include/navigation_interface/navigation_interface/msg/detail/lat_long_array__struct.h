// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from navigation_interface:msg/LatLongArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "navigation_interface/msg/lat_long_array.h"


#ifndef NAVIGATION_INTERFACE__MSG__DETAIL__LAT_LONG_ARRAY__STRUCT_H_
#define NAVIGATION_INTERFACE__MSG__DETAIL__LAT_LONG_ARRAY__STRUCT_H_

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
// Member 'gpspoints'
#include "navigation_interface/msg/detail/lat_long_point__struct.h"

/// Struct defined in msg/LatLongArray in the package navigation_interface.
typedef struct navigation_interface__msg__LatLongArray
{
  std_msgs__msg__Header header;
  navigation_interface__msg__LatLongPoint__Sequence gpspoints;
} navigation_interface__msg__LatLongArray;

// Struct for a sequence of navigation_interface__msg__LatLongArray.
typedef struct navigation_interface__msg__LatLongArray__Sequence
{
  navigation_interface__msg__LatLongArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} navigation_interface__msg__LatLongArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // NAVIGATION_INTERFACE__MSG__DETAIL__LAT_LONG_ARRAY__STRUCT_H_
