// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from navigation_interface:msg/VehicleState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "navigation_interface/msg/vehicle_state.hpp"


#ifndef NAVIGATION_INTERFACE__MSG__DETAIL__VEHICLE_STATE__BUILDER_HPP_
#define NAVIGATION_INTERFACE__MSG__DETAIL__VEHICLE_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "navigation_interface/msg/detail/vehicle_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace navigation_interface
{

namespace msg
{

namespace builder
{

class Init_VehicleState_stopped
{
public:
  explicit Init_VehicleState_stopped(::navigation_interface::msg::VehicleState & msg)
  : msg_(msg)
  {}
  ::navigation_interface::msg::VehicleState stopped(::navigation_interface::msg::VehicleState::_stopped_type arg)
  {
    msg_.stopped = std::move(arg);
    return std::move(msg_);
  }

private:
  ::navigation_interface::msg::VehicleState msg_;
};

class Init_VehicleState_reached_destination
{
public:
  explicit Init_VehicleState_reached_destination(::navigation_interface::msg::VehicleState & msg)
  : msg_(msg)
  {}
  Init_VehicleState_stopped reached_destination(::navigation_interface::msg::VehicleState::_reached_destination_type arg)
  {
    msg_.reached_destination = std::move(arg);
    return Init_VehicleState_stopped(msg_);
  }

private:
  ::navigation_interface::msg::VehicleState msg_;
};

class Init_VehicleState_is_navigating
{
public:
  explicit Init_VehicleState_is_navigating(::navigation_interface::msg::VehicleState & msg)
  : msg_(msg)
  {}
  Init_VehicleState_reached_destination is_navigating(::navigation_interface::msg::VehicleState::_is_navigating_type arg)
  {
    msg_.is_navigating = std::move(arg);
    return Init_VehicleState_reached_destination(msg_);
  }

private:
  ::navigation_interface::msg::VehicleState msg_;
};

class Init_VehicleState_header
{
public:
  Init_VehicleState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VehicleState_is_navigating header(::navigation_interface::msg::VehicleState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_VehicleState_is_navigating(msg_);
  }

private:
  ::navigation_interface::msg::VehicleState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::navigation_interface::msg::VehicleState>()
{
  return navigation_interface::msg::builder::Init_VehicleState_header();
}

}  // namespace navigation_interface

#endif  // NAVIGATION_INTERFACE__MSG__DETAIL__VEHICLE_STATE__BUILDER_HPP_
