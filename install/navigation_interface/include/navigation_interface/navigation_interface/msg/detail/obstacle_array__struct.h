// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from navigation_interface:msg/ObstacleArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "navigation_interface/msg/obstacle_array.h"


#ifndef NAVIGATION_INTERFACE__MSG__DETAIL__OBSTACLE_ARRAY__STRUCT_H_
#define NAVIGATION_INTERFACE__MSG__DETAIL__OBSTACLE_ARRAY__STRUCT_H_

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
// Member 'obstacles'
#include "navigation_interface/msg/detail/obstacle__struct.h"

/// Struct defined in msg/ObstacleArray in the package navigation_interface.
typedef struct navigation_interface__msg__ObstacleArray
{
  std_msgs__msg__Header header;
  navigation_interface__msg__Obstacle__Sequence obstacles;
} navigation_interface__msg__ObstacleArray;

// Struct for a sequence of navigation_interface__msg__ObstacleArray.
typedef struct navigation_interface__msg__ObstacleArray__Sequence
{
  navigation_interface__msg__ObstacleArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} navigation_interface__msg__ObstacleArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // NAVIGATION_INTERFACE__MSG__DETAIL__OBSTACLE_ARRAY__STRUCT_H_
