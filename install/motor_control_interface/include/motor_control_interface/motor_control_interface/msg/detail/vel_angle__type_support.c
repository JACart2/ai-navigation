// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from motor_control_interface:msg/VelAngle.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "motor_control_interface/msg/detail/vel_angle__rosidl_typesupport_introspection_c.h"
#include "motor_control_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "motor_control_interface/msg/detail/vel_angle__functions.h"
#include "motor_control_interface/msg/detail/vel_angle__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void motor_control_interface__msg__VelAngle__rosidl_typesupport_introspection_c__VelAngle_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  motor_control_interface__msg__VelAngle__init(message_memory);
}

void motor_control_interface__msg__VelAngle__rosidl_typesupport_introspection_c__VelAngle_fini_function(void * message_memory)
{
  motor_control_interface__msg__VelAngle__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember motor_control_interface__msg__VelAngle__rosidl_typesupport_introspection_c__VelAngle_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_control_interface__msg__VelAngle, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "vel",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_control_interface__msg__VelAngle, vel),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "angle",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_control_interface__msg__VelAngle, angle),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers motor_control_interface__msg__VelAngle__rosidl_typesupport_introspection_c__VelAngle_message_members = {
  "motor_control_interface__msg",  // message namespace
  "VelAngle",  // message name
  3,  // number of fields
  sizeof(motor_control_interface__msg__VelAngle),
  false,  // has_any_key_member_
  motor_control_interface__msg__VelAngle__rosidl_typesupport_introspection_c__VelAngle_message_member_array,  // message members
  motor_control_interface__msg__VelAngle__rosidl_typesupport_introspection_c__VelAngle_init_function,  // function to initialize message memory (memory has to be allocated)
  motor_control_interface__msg__VelAngle__rosidl_typesupport_introspection_c__VelAngle_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t motor_control_interface__msg__VelAngle__rosidl_typesupport_introspection_c__VelAngle_message_type_support_handle = {
  0,
  &motor_control_interface__msg__VelAngle__rosidl_typesupport_introspection_c__VelAngle_message_members,
  get_message_typesupport_handle_function,
  &motor_control_interface__msg__VelAngle__get_type_hash,
  &motor_control_interface__msg__VelAngle__get_type_description,
  &motor_control_interface__msg__VelAngle__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_motor_control_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, motor_control_interface, msg, VelAngle)() {
  motor_control_interface__msg__VelAngle__rosidl_typesupport_introspection_c__VelAngle_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!motor_control_interface__msg__VelAngle__rosidl_typesupport_introspection_c__VelAngle_message_type_support_handle.typesupport_identifier) {
    motor_control_interface__msg__VelAngle__rosidl_typesupport_introspection_c__VelAngle_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &motor_control_interface__msg__VelAngle__rosidl_typesupport_introspection_c__VelAngle_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
