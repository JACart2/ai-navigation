// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from navigation_interface:msg/ObstacleArray.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "navigation_interface/msg/detail/obstacle_array__functions.h"
#include "navigation_interface/msg/detail/obstacle_array__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace navigation_interface
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void ObstacleArray_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) navigation_interface::msg::ObstacleArray(_init);
}

void ObstacleArray_fini_function(void * message_memory)
{
  auto typed_message = static_cast<navigation_interface::msg::ObstacleArray *>(message_memory);
  typed_message->~ObstacleArray();
}

size_t size_function__ObstacleArray__obstacles(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<navigation_interface::msg::Obstacle> *>(untyped_member);
  return member->size();
}

const void * get_const_function__ObstacleArray__obstacles(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<navigation_interface::msg::Obstacle> *>(untyped_member);
  return &member[index];
}

void * get_function__ObstacleArray__obstacles(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<navigation_interface::msg::Obstacle> *>(untyped_member);
  return &member[index];
}

void fetch_function__ObstacleArray__obstacles(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const navigation_interface::msg::Obstacle *>(
    get_const_function__ObstacleArray__obstacles(untyped_member, index));
  auto & value = *reinterpret_cast<navigation_interface::msg::Obstacle *>(untyped_value);
  value = item;
}

void assign_function__ObstacleArray__obstacles(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<navigation_interface::msg::Obstacle *>(
    get_function__ObstacleArray__obstacles(untyped_member, index));
  const auto & value = *reinterpret_cast<const navigation_interface::msg::Obstacle *>(untyped_value);
  item = value;
}

void resize_function__ObstacleArray__obstacles(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<navigation_interface::msg::Obstacle> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ObstacleArray_message_member_array[2] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(navigation_interface::msg::ObstacleArray, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "obstacles",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<navigation_interface::msg::Obstacle>(),  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(navigation_interface::msg::ObstacleArray, obstacles),  // bytes offset in struct
    nullptr,  // default value
    size_function__ObstacleArray__obstacles,  // size() function pointer
    get_const_function__ObstacleArray__obstacles,  // get_const(index) function pointer
    get_function__ObstacleArray__obstacles,  // get(index) function pointer
    fetch_function__ObstacleArray__obstacles,  // fetch(index, &value) function pointer
    assign_function__ObstacleArray__obstacles,  // assign(index, value) function pointer
    resize_function__ObstacleArray__obstacles  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ObstacleArray_message_members = {
  "navigation_interface::msg",  // message namespace
  "ObstacleArray",  // message name
  2,  // number of fields
  sizeof(navigation_interface::msg::ObstacleArray),
  false,  // has_any_key_member_
  ObstacleArray_message_member_array,  // message members
  ObstacleArray_init_function,  // function to initialize message memory (memory has to be allocated)
  ObstacleArray_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ObstacleArray_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ObstacleArray_message_members,
  get_message_typesupport_handle_function,
  &navigation_interface__msg__ObstacleArray__get_type_hash,
  &navigation_interface__msg__ObstacleArray__get_type_description,
  &navigation_interface__msg__ObstacleArray__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace navigation_interface


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<navigation_interface::msg::ObstacleArray>()
{
  return &::navigation_interface::msg::rosidl_typesupport_introspection_cpp::ObstacleArray_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, navigation_interface, msg, ObstacleArray)() {
  return &::navigation_interface::msg::rosidl_typesupport_introspection_cpp::ObstacleArray_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
