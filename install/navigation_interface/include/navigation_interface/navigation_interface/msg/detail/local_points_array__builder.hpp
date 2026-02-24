// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from navigation_interface:msg/LocalPointsArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "navigation_interface/msg/local_points_array.hpp"


#ifndef NAVIGATION_INTERFACE__MSG__DETAIL__LOCAL_POINTS_ARRAY__BUILDER_HPP_
#define NAVIGATION_INTERFACE__MSG__DETAIL__LOCAL_POINTS_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "navigation_interface/msg/detail/local_points_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace navigation_interface
{

namespace msg
{

namespace builder
{

class Init_LocalPointsArray_total_distance
{
public:
  explicit Init_LocalPointsArray_total_distance(::navigation_interface::msg::LocalPointsArray & msg)
  : msg_(msg)
  {}
  ::navigation_interface::msg::LocalPointsArray total_distance(::navigation_interface::msg::LocalPointsArray::_total_distance_type arg)
  {
    msg_.total_distance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::navigation_interface::msg::LocalPointsArray msg_;
};

class Init_LocalPointsArray_localpoints
{
public:
  explicit Init_LocalPointsArray_localpoints(::navigation_interface::msg::LocalPointsArray & msg)
  : msg_(msg)
  {}
  Init_LocalPointsArray_total_distance localpoints(::navigation_interface::msg::LocalPointsArray::_localpoints_type arg)
  {
    msg_.localpoints = std::move(arg);
    return Init_LocalPointsArray_total_distance(msg_);
  }

private:
  ::navigation_interface::msg::LocalPointsArray msg_;
};

class Init_LocalPointsArray_header
{
public:
  Init_LocalPointsArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LocalPointsArray_localpoints header(::navigation_interface::msg::LocalPointsArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_LocalPointsArray_localpoints(msg_);
  }

private:
  ::navigation_interface::msg::LocalPointsArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::navigation_interface::msg::LocalPointsArray>()
{
  return navigation_interface::msg::builder::Init_LocalPointsArray_header();
}

}  // namespace navigation_interface

#endif  // NAVIGATION_INTERFACE__MSG__DETAIL__LOCAL_POINTS_ARRAY__BUILDER_HPP_
