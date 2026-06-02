// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from navigation_interface:msg/Obstacle.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "navigation_interface/msg/obstacle.h"


#ifndef NAVIGATION_INTERFACE__MSG__DETAIL__OBSTACLE__STRUCT_H_
#define NAVIGATION_INTERFACE__MSG__DETAIL__OBSTACLE__STRUCT_H_

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
// Member 'pos'
#include "geometry_msgs/msg/detail/point_stamped__struct.h"

/// Struct defined in msg/Obstacle in the package navigation_interface.
typedef struct navigation_interface__msg__Obstacle
{
  std_msgs__msg__Header header;
  geometry_msgs__msg__PointStamped pos;
  double radius;
  bool followable;
} navigation_interface__msg__Obstacle;

// Struct for a sequence of navigation_interface__msg__Obstacle.
typedef struct navigation_interface__msg__Obstacle__Sequence
{
  navigation_interface__msg__Obstacle * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} navigation_interface__msg__Obstacle__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // NAVIGATION_INTERFACE__MSG__DETAIL__OBSTACLE__STRUCT_H_
