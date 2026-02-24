// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from navigation_interface:msg/LatLongPoint.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "navigation_interface/msg/lat_long_point.hpp"


#ifndef NAVIGATION_INTERFACE__MSG__DETAIL__LAT_LONG_POINT__BUILDER_HPP_
#define NAVIGATION_INTERFACE__MSG__DETAIL__LAT_LONG_POINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "navigation_interface/msg/detail/lat_long_point__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace navigation_interface
{

namespace msg
{

namespace builder
{

class Init_LatLongPoint_elevation
{
public:
  explicit Init_LatLongPoint_elevation(::navigation_interface::msg::LatLongPoint & msg)
  : msg_(msg)
  {}
  ::navigation_interface::msg::LatLongPoint elevation(::navigation_interface::msg::LatLongPoint::_elevation_type arg)
  {
    msg_.elevation = std::move(arg);
    return std::move(msg_);
  }

private:
  ::navigation_interface::msg::LatLongPoint msg_;
};

class Init_LatLongPoint_longitude
{
public:
  explicit Init_LatLongPoint_longitude(::navigation_interface::msg::LatLongPoint & msg)
  : msg_(msg)
  {}
  Init_LatLongPoint_elevation longitude(::navigation_interface::msg::LatLongPoint::_longitude_type arg)
  {
    msg_.longitude = std::move(arg);
    return Init_LatLongPoint_elevation(msg_);
  }

private:
  ::navigation_interface::msg::LatLongPoint msg_;
};

class Init_LatLongPoint_latitude
{
public:
  explicit Init_LatLongPoint_latitude(::navigation_interface::msg::LatLongPoint & msg)
  : msg_(msg)
  {}
  Init_LatLongPoint_longitude latitude(::navigation_interface::msg::LatLongPoint::_latitude_type arg)
  {
    msg_.latitude = std::move(arg);
    return Init_LatLongPoint_longitude(msg_);
  }

private:
  ::navigation_interface::msg::LatLongPoint msg_;
};

class Init_LatLongPoint_header
{
public:
  Init_LatLongPoint_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LatLongPoint_latitude header(::navigation_interface::msg::LatLongPoint::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_LatLongPoint_latitude(msg_);
  }

private:
  ::navigation_interface::msg::LatLongPoint msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::navigation_interface::msg::LatLongPoint>()
{
  return navigation_interface::msg::builder::Init_LatLongPoint_header();
}

}  // namespace navigation_interface

#endif  // NAVIGATION_INTERFACE__MSG__DETAIL__LAT_LONG_POINT__BUILDER_HPP_
