// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from navigation_interface:msg/WaypointsArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "navigation_interface/msg/detail/waypoints_array__rosidl_typesupport_introspection_c.h"
#include "navigation_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "navigation_interface/msg/detail/waypoints_array__functions.h"
#include "navigation_interface/msg/detail/waypoints_array__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `waypoints`
#include "sensor_msgs/msg/nav_sat_fix.h"
// Member `waypoints`
#include "sensor_msgs/msg/detail/nav_sat_fix__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void navigation_interface__msg__WaypointsArray__rosidl_typesupport_introspection_c__WaypointsArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  navigation_interface__msg__WaypointsArray__init(message_memory);
}

void navigation_interface__msg__WaypointsArray__rosidl_typesupport_introspection_c__WaypointsArray_fini_function(void * message_memory)
{
  navigation_interface__msg__WaypointsArray__fini(message_memory);
}

size_t navigation_interface__msg__WaypointsArray__rosidl_typesupport_introspection_c__size_function__WaypointsArray__waypoints(
  const void * untyped_member)
{
  const sensor_msgs__msg__NavSatFix__Sequence * member =
    (const sensor_msgs__msg__NavSatFix__Sequence *)(untyped_member);
  return member->size;
}

const void * navigation_interface__msg__WaypointsArray__rosidl_typesupport_introspection_c__get_const_function__WaypointsArray__waypoints(
  const void * untyped_member, size_t index)
{
  const sensor_msgs__msg__NavSatFix__Sequence * member =
    (const sensor_msgs__msg__NavSatFix__Sequence *)(untyped_member);
  return &member->data[index];
}

void * navigation_interface__msg__WaypointsArray__rosidl_typesupport_introspection_c__get_function__WaypointsArray__waypoints(
  void * untyped_member, size_t index)
{
  sensor_msgs__msg__NavSatFix__Sequence * member =
    (sensor_msgs__msg__NavSatFix__Sequence *)(untyped_member);
  return &member->data[index];
}

void navigation_interface__msg__WaypointsArray__rosidl_typesupport_introspection_c__fetch_function__WaypointsArray__waypoints(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const sensor_msgs__msg__NavSatFix * item =
    ((const sensor_msgs__msg__NavSatFix *)
    navigation_interface__msg__WaypointsArray__rosidl_typesupport_introspection_c__get_const_function__WaypointsArray__waypoints(untyped_member, index));
  sensor_msgs__msg__NavSatFix * value =
    (sensor_msgs__msg__NavSatFix *)(untyped_value);
  *value = *item;
}

void navigation_interface__msg__WaypointsArray__rosidl_typesupport_introspection_c__assign_function__WaypointsArray__waypoints(
  void * untyped_member, size_t index, const void * untyped_value)
{
  sensor_msgs__msg__NavSatFix * item =
    ((sensor_msgs__msg__NavSatFix *)
    navigation_interface__msg__WaypointsArray__rosidl_typesupport_introspection_c__get_function__WaypointsArray__waypoints(untyped_member, index));
  const sensor_msgs__msg__NavSatFix * value =
    (const sensor_msgs__msg__NavSatFix *)(untyped_value);
  *item = *value;
}

bool navigation_interface__msg__WaypointsArray__rosidl_typesupport_introspection_c__resize_function__WaypointsArray__waypoints(
  void * untyped_member, size_t size)
{
  sensor_msgs__msg__NavSatFix__Sequence * member =
    (sensor_msgs__msg__NavSatFix__Sequence *)(untyped_member);
  sensor_msgs__msg__NavSatFix__Sequence__fini(member);
  return sensor_msgs__msg__NavSatFix__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember navigation_interface__msg__WaypointsArray__rosidl_typesupport_introspection_c__WaypointsArray_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(navigation_interface__msg__WaypointsArray, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "waypoints",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(navigation_interface__msg__WaypointsArray, waypoints),  // bytes offset in struct
    NULL,  // default value
    navigation_interface__msg__WaypointsArray__rosidl_typesupport_introspection_c__size_function__WaypointsArray__waypoints,  // size() function pointer
    navigation_interface__msg__WaypointsArray__rosidl_typesupport_introspection_c__get_const_function__WaypointsArray__waypoints,  // get_const(index) function pointer
    navigation_interface__msg__WaypointsArray__rosidl_typesupport_introspection_c__get_function__WaypointsArray__waypoints,  // get(index) function pointer
    navigation_interface__msg__WaypointsArray__rosidl_typesupport_introspection_c__fetch_function__WaypointsArray__waypoints,  // fetch(index, &value) function pointer
    navigation_interface__msg__WaypointsArray__rosidl_typesupport_introspection_c__assign_function__WaypointsArray__waypoints,  // assign(index, value) function pointer
    navigation_interface__msg__WaypointsArray__rosidl_typesupport_introspection_c__resize_function__WaypointsArray__waypoints  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers navigation_interface__msg__WaypointsArray__rosidl_typesupport_introspection_c__WaypointsArray_message_members = {
  "navigation_interface__msg",  // message namespace
  "WaypointsArray",  // message name
  2,  // number of fields
  sizeof(navigation_interface__msg__WaypointsArray),
  false,  // has_any_key_member_
  navigation_interface__msg__WaypointsArray__rosidl_typesupport_introspection_c__WaypointsArray_message_member_array,  // message members
  navigation_interface__msg__WaypointsArray__rosidl_typesupport_introspection_c__WaypointsArray_init_function,  // function to initialize message memory (memory has to be allocated)
  navigation_interface__msg__WaypointsArray__rosidl_typesupport_introspection_c__WaypointsArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t navigation_interface__msg__WaypointsArray__rosidl_typesupport_introspection_c__WaypointsArray_message_type_support_handle = {
  0,
  &navigation_interface__msg__WaypointsArray__rosidl_typesupport_introspection_c__WaypointsArray_message_members,
  get_message_typesupport_handle_function,
  &navigation_interface__msg__WaypointsArray__get_type_hash,
  &navigation_interface__msg__WaypointsArray__get_type_description,
  &navigation_interface__msg__WaypointsArray__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_navigation_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, navigation_interface, msg, WaypointsArray)() {
  navigation_interface__msg__WaypointsArray__rosidl_typesupport_introspection_c__WaypointsArray_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  navigation_interface__msg__WaypointsArray__rosidl_typesupport_introspection_c__WaypointsArray_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, NavSatFix)();
  if (!navigation_interface__msg__WaypointsArray__rosidl_typesupport_introspection_c__WaypointsArray_message_type_support_handle.typesupport_identifier) {
    navigation_interface__msg__WaypointsArray__rosidl_typesupport_introspection_c__WaypointsArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &navigation_interface__msg__WaypointsArray__rosidl_typesupport_introspection_c__WaypointsArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
