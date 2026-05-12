// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from navigation_interface:msg/LocalPointsArray.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "navigation_interface/msg/detail/local_points_array__functions.h"
#include "navigation_interface/msg/detail/local_points_array__struct.hpp"
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

void LocalPointsArray_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) navigation_interface::msg::LocalPointsArray(_init);
}

void LocalPointsArray_fini_function(void * message_memory)
{
  auto typed_message = static_cast<navigation_interface::msg::LocalPointsArray *>(message_memory);
  typed_message->~LocalPointsArray();
}

size_t size_function__LocalPointsArray__localpoints(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<geometry_msgs::msg::Pose> *>(untyped_member);
  return member->size();
}

const void * get_const_function__LocalPointsArray__localpoints(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<geometry_msgs::msg::Pose> *>(untyped_member);
  return &member[index];
}

void * get_function__LocalPointsArray__localpoints(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<geometry_msgs::msg::Pose> *>(untyped_member);
  return &member[index];
}

void fetch_function__LocalPointsArray__localpoints(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const geometry_msgs::msg::Pose *>(
    get_const_function__LocalPointsArray__localpoints(untyped_member, index));
  auto & value = *reinterpret_cast<geometry_msgs::msg::Pose *>(untyped_value);
  value = item;
}

void assign_function__LocalPointsArray__localpoints(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<geometry_msgs::msg::Pose *>(
    get_function__LocalPointsArray__localpoints(untyped_member, index));
  const auto & value = *reinterpret_cast<const geometry_msgs::msg::Pose *>(untyped_value);
  item = value;
}

void resize_function__LocalPointsArray__localpoints(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<geometry_msgs::msg::Pose> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember LocalPointsArray_message_member_array[3] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(navigation_interface::msg::LocalPointsArray, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "localpoints",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Pose>(),  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(navigation_interface::msg::LocalPointsArray, localpoints),  // bytes offset in struct
    nullptr,  // default value
    size_function__LocalPointsArray__localpoints,  // size() function pointer
    get_const_function__LocalPointsArray__localpoints,  // get_const(index) function pointer
    get_function__LocalPointsArray__localpoints,  // get(index) function pointer
    fetch_function__LocalPointsArray__localpoints,  // fetch(index, &value) function pointer
    assign_function__LocalPointsArray__localpoints,  // assign(index, value) function pointer
    resize_function__LocalPointsArray__localpoints  // resize(index) function pointer
  },
  {
    "total_distance",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Float64>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(navigation_interface::msg::LocalPointsArray, total_distance),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers LocalPointsArray_message_members = {
  "navigation_interface::msg",  // message namespace
  "LocalPointsArray",  // message name
  3,  // number of fields
  sizeof(navigation_interface::msg::LocalPointsArray),
  false,  // has_any_key_member_
  LocalPointsArray_message_member_array,  // message members
  LocalPointsArray_init_function,  // function to initialize message memory (memory has to be allocated)
  LocalPointsArray_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t LocalPointsArray_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &LocalPointsArray_message_members,
  get_message_typesupport_handle_function,
  &navigation_interface__msg__LocalPointsArray__get_type_hash,
  &navigation_interface__msg__LocalPointsArray__get_type_description,
  &navigation_interface__msg__LocalPointsArray__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace navigation_interface


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<navigation_interface::msg::LocalPointsArray>()
{
  return &::navigation_interface::msg::rosidl_typesupport_introspection_cpp::LocalPointsArray_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, navigation_interface, msg, LocalPointsArray)() {
  return &::navigation_interface::msg::rosidl_typesupport_introspection_cpp::LocalPointsArray_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
