// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from navigation_interface:msg/LocalPointsArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "navigation_interface/msg/detail/local_points_array__rosidl_typesupport_introspection_c.h"
#include "navigation_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "navigation_interface/msg/detail/local_points_array__functions.h"
#include "navigation_interface/msg/detail/local_points_array__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `localpoints`
#include "geometry_msgs/msg/pose.h"
// Member `localpoints`
#include "geometry_msgs/msg/detail/pose__rosidl_typesupport_introspection_c.h"
// Member `total_distance`
#include "std_msgs/msg/float64.h"
// Member `total_distance`
#include "std_msgs/msg/detail/float64__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void navigation_interface__msg__LocalPointsArray__rosidl_typesupport_introspection_c__LocalPointsArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  navigation_interface__msg__LocalPointsArray__init(message_memory);
}

void navigation_interface__msg__LocalPointsArray__rosidl_typesupport_introspection_c__LocalPointsArray_fini_function(void * message_memory)
{
  navigation_interface__msg__LocalPointsArray__fini(message_memory);
}

size_t navigation_interface__msg__LocalPointsArray__rosidl_typesupport_introspection_c__size_function__LocalPointsArray__localpoints(
  const void * untyped_member)
{
  const geometry_msgs__msg__Pose__Sequence * member =
    (const geometry_msgs__msg__Pose__Sequence *)(untyped_member);
  return member->size;
}

const void * navigation_interface__msg__LocalPointsArray__rosidl_typesupport_introspection_c__get_const_function__LocalPointsArray__localpoints(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__Pose__Sequence * member =
    (const geometry_msgs__msg__Pose__Sequence *)(untyped_member);
  return &member->data[index];
}

void * navigation_interface__msg__LocalPointsArray__rosidl_typesupport_introspection_c__get_function__LocalPointsArray__localpoints(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__Pose__Sequence * member =
    (geometry_msgs__msg__Pose__Sequence *)(untyped_member);
  return &member->data[index];
}

void navigation_interface__msg__LocalPointsArray__rosidl_typesupport_introspection_c__fetch_function__LocalPointsArray__localpoints(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const geometry_msgs__msg__Pose * item =
    ((const geometry_msgs__msg__Pose *)
    navigation_interface__msg__LocalPointsArray__rosidl_typesupport_introspection_c__get_const_function__LocalPointsArray__localpoints(untyped_member, index));
  geometry_msgs__msg__Pose * value =
    (geometry_msgs__msg__Pose *)(untyped_value);
  *value = *item;
}

void navigation_interface__msg__LocalPointsArray__rosidl_typesupport_introspection_c__assign_function__LocalPointsArray__localpoints(
  void * untyped_member, size_t index, const void * untyped_value)
{
  geometry_msgs__msg__Pose * item =
    ((geometry_msgs__msg__Pose *)
    navigation_interface__msg__LocalPointsArray__rosidl_typesupport_introspection_c__get_function__LocalPointsArray__localpoints(untyped_member, index));
  const geometry_msgs__msg__Pose * value =
    (const geometry_msgs__msg__Pose *)(untyped_value);
  *item = *value;
}

bool navigation_interface__msg__LocalPointsArray__rosidl_typesupport_introspection_c__resize_function__LocalPointsArray__localpoints(
  void * untyped_member, size_t size)
{
  geometry_msgs__msg__Pose__Sequence * member =
    (geometry_msgs__msg__Pose__Sequence *)(untyped_member);
  geometry_msgs__msg__Pose__Sequence__fini(member);
  return geometry_msgs__msg__Pose__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember navigation_interface__msg__LocalPointsArray__rosidl_typesupport_introspection_c__LocalPointsArray_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(navigation_interface__msg__LocalPointsArray, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "localpoints",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(navigation_interface__msg__LocalPointsArray, localpoints),  // bytes offset in struct
    NULL,  // default value
    navigation_interface__msg__LocalPointsArray__rosidl_typesupport_introspection_c__size_function__LocalPointsArray__localpoints,  // size() function pointer
    navigation_interface__msg__LocalPointsArray__rosidl_typesupport_introspection_c__get_const_function__LocalPointsArray__localpoints,  // get_const(index) function pointer
    navigation_interface__msg__LocalPointsArray__rosidl_typesupport_introspection_c__get_function__LocalPointsArray__localpoints,  // get(index) function pointer
    navigation_interface__msg__LocalPointsArray__rosidl_typesupport_introspection_c__fetch_function__LocalPointsArray__localpoints,  // fetch(index, &value) function pointer
    navigation_interface__msg__LocalPointsArray__rosidl_typesupport_introspection_c__assign_function__LocalPointsArray__localpoints,  // assign(index, value) function pointer
    navigation_interface__msg__LocalPointsArray__rosidl_typesupport_introspection_c__resize_function__LocalPointsArray__localpoints  // resize(index) function pointer
  },
  {
    "total_distance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(navigation_interface__msg__LocalPointsArray, total_distance),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers navigation_interface__msg__LocalPointsArray__rosidl_typesupport_introspection_c__LocalPointsArray_message_members = {
  "navigation_interface__msg",  // message namespace
  "LocalPointsArray",  // message name
  3,  // number of fields
  sizeof(navigation_interface__msg__LocalPointsArray),
  false,  // has_any_key_member_
  navigation_interface__msg__LocalPointsArray__rosidl_typesupport_introspection_c__LocalPointsArray_message_member_array,  // message members
  navigation_interface__msg__LocalPointsArray__rosidl_typesupport_introspection_c__LocalPointsArray_init_function,  // function to initialize message memory (memory has to be allocated)
  navigation_interface__msg__LocalPointsArray__rosidl_typesupport_introspection_c__LocalPointsArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t navigation_interface__msg__LocalPointsArray__rosidl_typesupport_introspection_c__LocalPointsArray_message_type_support_handle = {
  0,
  &navigation_interface__msg__LocalPointsArray__rosidl_typesupport_introspection_c__LocalPointsArray_message_members,
  get_message_typesupport_handle_function,
  &navigation_interface__msg__LocalPointsArray__get_type_hash,
  &navigation_interface__msg__LocalPointsArray__get_type_description,
  &navigation_interface__msg__LocalPointsArray__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_navigation_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, navigation_interface, msg, LocalPointsArray)() {
  navigation_interface__msg__LocalPointsArray__rosidl_typesupport_introspection_c__LocalPointsArray_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  navigation_interface__msg__LocalPointsArray__rosidl_typesupport_introspection_c__LocalPointsArray_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  navigation_interface__msg__LocalPointsArray__rosidl_typesupport_introspection_c__LocalPointsArray_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Float64)();
  if (!navigation_interface__msg__LocalPointsArray__rosidl_typesupport_introspection_c__LocalPointsArray_message_type_support_handle.typesupport_identifier) {
    navigation_interface__msg__LocalPointsArray__rosidl_typesupport_introspection_c__LocalPointsArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &navigation_interface__msg__LocalPointsArray__rosidl_typesupport_introspection_c__LocalPointsArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
