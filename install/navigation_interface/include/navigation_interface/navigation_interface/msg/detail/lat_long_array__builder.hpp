// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from navigation_interface:msg/LatLongArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "navigation_interface/msg/lat_long_array.hpp"


#ifndef NAVIGATION_INTERFACE__MSG__DETAIL__LAT_LONG_ARRAY__BUILDER_HPP_
#define NAVIGATION_INTERFACE__MSG__DETAIL__LAT_LONG_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "navigation_interface/msg/detail/lat_long_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace navigation_interface
{

namespace msg
{

namespace builder
{

class Init_LatLongArray_gpspoints
{
public:
  explicit Init_LatLongArray_gpspoints(::navigation_interface::msg::LatLongArray & msg)
  : msg_(msg)
  {}
  ::navigation_interface::msg::LatLongArray gpspoints(::navigation_interface::msg::LatLongArray::_gpspoints_type arg)
  {
    msg_.gpspoints = std::move(arg);
    return std::move(msg_);
  }

private:
  ::navigation_interface::msg::LatLongArray msg_;
};

class Init_LatLongArray_header
{
public:
  Init_LatLongArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LatLongArray_gpspoints header(::navigation_interface::msg::LatLongArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_LatLongArray_gpspoints(msg_);
  }

private:
  ::navigation_interface::msg::LatLongArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::navigation_interface::msg::LatLongArray>()
{
  return navigation_interface::msg::builder::Init_LatLongArray_header();
}

}  // namespace navigation_interface

#endif  // NAVIGATION_INTERFACE__MSG__DETAIL__LAT_LONG_ARRAY__BUILDER_HPP_
