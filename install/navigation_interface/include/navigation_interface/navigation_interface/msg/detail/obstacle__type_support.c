// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from navigation_interface:msg/Obstacle.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "navigation_interface/msg/detail/obstacle__rosidl_typesupport_introspection_c.h"
#include "navigation_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "navigation_interface/msg/detail/obstacle__functions.h"
#include "navigation_interface/msg/detail/obstacle__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `pos`
#include "geometry_msgs/msg/point_stamped.h"
// Member `pos`
#include "geometry_msgs/msg/detail/point_stamped__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void navigation_interface__msg__Obstacle__rosidl_typesupport_introspection_c__Obstacle_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  navigation_interface__msg__Obstacle__init(message_memory);
}

void navigation_interface__msg__Obstacle__rosidl_typesupport_introspection_c__Obstacle_fini_function(void * message_memory)
{
  navigation_interface__msg__Obstacle__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember navigation_interface__msg__Obstacle__rosidl_typesupport_introspection_c__Obstacle_message_member_array[4] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(navigation_interface__msg__Obstacle, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pos",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(navigation_interface__msg__Obstacle, pos),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "radius",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(navigation_interface__msg__Obstacle, radius),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "followable",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(navigation_interface__msg__Obstacle, followable),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers navigation_interface__msg__Obstacle__rosidl_typesupport_introspection_c__Obstacle_message_members = {
  "navigation_interface__msg",  // message namespace
  "Obstacle",  // message name
  4,  // number of fields
  sizeof(navigation_interface__msg__Obstacle),
  false,  // has_any_key_member_
  navigation_interface__msg__Obstacle__rosidl_typesupport_introspection_c__Obstacle_message_member_array,  // message members
  navigation_interface__msg__Obstacle__rosidl_typesupport_introspection_c__Obstacle_init_function,  // function to initialize message memory (memory has to be allocated)
  navigation_interface__msg__Obstacle__rosidl_typesupport_introspection_c__Obstacle_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t navigation_interface__msg__Obstacle__rosidl_typesupport_introspection_c__Obstacle_message_type_support_handle = {
  0,
  &navigation_interface__msg__Obstacle__rosidl_typesupport_introspection_c__Obstacle_message_members,
  get_message_typesupport_handle_function,
  &navigation_interface__msg__Obstacle__get_type_hash,
  &navigation_interface__msg__Obstacle__get_type_description,
  &navigation_interface__msg__Obstacle__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_navigation_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, navigation_interface, msg, Obstacle)() {
  navigation_interface__msg__Obstacle__rosidl_typesupport_introspection_c__Obstacle_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  navigation_interface__msg__Obstacle__rosidl_typesupport_introspection_c__Obstacle_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PointStamped)();
  if (!navigation_interface__msg__Obstacle__rosidl_typesupport_introspection_c__Obstacle_message_type_support_handle.typesupport_identifier) {
    navigation_interface__msg__Obstacle__rosidl_typesupport_introspection_c__Obstacle_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &navigation_interface__msg__Obstacle__rosidl_typesupport_introspection_c__Obstacle_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
