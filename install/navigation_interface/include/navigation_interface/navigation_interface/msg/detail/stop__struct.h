// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from navigation_interface:msg/Stop.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "navigation_interface/msg/stop.h"


#ifndef NAVIGATION_INTERFACE__MSG__DETAIL__STOP__STRUCT_H_
#define NAVIGATION_INTERFACE__MSG__DETAIL__STOP__STRUCT_H_

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
// Member 'sender_id'
#include "std_msgs/msg/detail/string__struct.h"

/// Struct defined in msg/Stop in the package navigation_interface.
typedef struct navigation_interface__msg__Stop
{
  std_msgs__msg__Header header;
  bool stop;
  std_msgs__msg__String sender_id;
  double distance;
} navigation_interface__msg__Stop;

// Struct for a sequence of navigation_interface__msg__Stop.
typedef struct navigation_interface__msg__Stop__Sequence
{
  navigation_interface__msg__Stop * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} navigation_interface__msg__Stop__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // NAVIGATION_INTERFACE__MSG__DETAIL__STOP__STRUCT_H_
