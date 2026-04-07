// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from navigation_interface:msg/Stop.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "navigation_interface/msg/stop.hpp"


#ifndef NAVIGATION_INTERFACE__MSG__DETAIL__STOP__BUILDER_HPP_
#define NAVIGATION_INTERFACE__MSG__DETAIL__STOP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "navigation_interface/msg/detail/stop__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace navigation_interface
{

namespace msg
{

namespace builder
{

class Init_Stop_distance
{
public:
  explicit Init_Stop_distance(::navigation_interface::msg::Stop & msg)
  : msg_(msg)
  {}
  ::navigation_interface::msg::Stop distance(::navigation_interface::msg::Stop::_distance_type arg)
  {
    msg_.distance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::navigation_interface::msg::Stop msg_;
};

class Init_Stop_sender_id
{
public:
  explicit Init_Stop_sender_id(::navigation_interface::msg::Stop & msg)
  : msg_(msg)
  {}
  Init_Stop_distance sender_id(::navigation_interface::msg::Stop::_sender_id_type arg)
  {
    msg_.sender_id = std::move(arg);
    return Init_Stop_distance(msg_);
  }

private:
  ::navigation_interface::msg::Stop msg_;
};

class Init_Stop_stop
{
public:
  explicit Init_Stop_stop(::navigation_interface::msg::Stop & msg)
  : msg_(msg)
  {}
  Init_Stop_sender_id stop(::navigation_interface::msg::Stop::_stop_type arg)
  {
    msg_.stop = std::move(arg);
    return Init_Stop_sender_id(msg_);
  }

private:
  ::navigation_interface::msg::Stop msg_;
};

class Init_Stop_header
{
public:
  Init_Stop_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Stop_stop header(::navigation_interface::msg::Stop::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Stop_stop(msg_);
  }

private:
  ::navigation_interface::msg::Stop msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::navigation_interface::msg::Stop>()
{
  return navigation_interface::msg::builder::Init_Stop_header();
}

}  // namespace navigation_interface

#endif  // NAVIGATION_INTERFACE__MSG__DETAIL__STOP__BUILDER_HPP_
