// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from navigation_interface:msg/LocalPointsArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "navigation_interface/msg/local_points_array.h"


#ifndef NAVIGATION_INTERFACE__MSG__DETAIL__LOCAL_POINTS_ARRAY__STRUCT_H_
#define NAVIGATION_INTERFACE__MSG__DETAIL__LOCAL_POINTS_ARRAY__STRUCT_H_

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
// Member 'localpoints'
#include "geometry_msgs/msg/detail/pose__struct.h"
// Member 'total_distance'
#include "std_msgs/msg/detail/float64__struct.h"

/// Struct defined in msg/LocalPointsArray in the package navigation_interface.
typedef struct navigation_interface__msg__LocalPointsArray
{
  std_msgs__msg__Header header;
  geometry_msgs__msg__Pose__Sequence localpoints;
  std_msgs__msg__Float64 total_distance;
} navigation_interface__msg__LocalPointsArray;

// Struct for a sequence of navigation_interface__msg__LocalPointsArray.
typedef struct navigation_interface__msg__LocalPointsArray__Sequence
{
  navigation_interface__msg__LocalPointsArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} navigation_interface__msg__LocalPointsArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // NAVIGATION_INTERFACE__MSG__DETAIL__LOCAL_POINTS_ARRAY__STRUCT_H_
