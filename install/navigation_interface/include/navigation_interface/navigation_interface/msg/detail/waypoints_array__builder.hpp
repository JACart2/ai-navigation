// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from navigation_interface:msg/WaypointsArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "navigation_interface/msg/waypoints_array.hpp"


#ifndef NAVIGATION_INTERFACE__MSG__DETAIL__WAYPOINTS_ARRAY__BUILDER_HPP_
#define NAVIGATION_INTERFACE__MSG__DETAIL__WAYPOINTS_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "navigation_interface/msg/detail/waypoints_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace navigation_interface
{

namespace msg
{

namespace builder
{

class Init_WaypointsArray_waypoints
{
public:
  explicit Init_WaypointsArray_waypoints(::navigation_interface::msg::WaypointsArray & msg)
  : msg_(msg)
  {}
  ::navigation_interface::msg::WaypointsArray waypoints(::navigation_interface::msg::WaypointsArray::_waypoints_type arg)
  {
    msg_.waypoints = std::move(arg);
    return std::move(msg_);
  }

private:
  ::navigation_interface::msg::WaypointsArray msg_;
};

class Init_WaypointsArray_header
{
public:
  Init_WaypointsArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WaypointsArray_waypoints header(::navigation_interface::msg::WaypointsArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_WaypointsArray_waypoints(msg_);
  }

private:
  ::navigation_interface::msg::WaypointsArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::navigation_interface::msg::WaypointsArray>()
{
  return navigation_interface::msg::builder::Init_WaypointsArray_header();
}

}  // namespace navigation_interface

#endif  // NAVIGATION_INTERFACE__MSG__DETAIL__WAYPOINTS_ARRAY__BUILDER_HPP_
