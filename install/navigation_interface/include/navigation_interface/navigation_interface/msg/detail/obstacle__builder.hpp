// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from navigation_interface:msg/Obstacle.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "navigation_interface/msg/obstacle.hpp"


#ifndef NAVIGATION_INTERFACE__MSG__DETAIL__OBSTACLE__BUILDER_HPP_
#define NAVIGATION_INTERFACE__MSG__DETAIL__OBSTACLE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "navigation_interface/msg/detail/obstacle__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace navigation_interface
{

namespace msg
{

namespace builder
{

class Init_Obstacle_followable
{
public:
  explicit Init_Obstacle_followable(::navigation_interface::msg::Obstacle & msg)
  : msg_(msg)
  {}
  ::navigation_interface::msg::Obstacle followable(::navigation_interface::msg::Obstacle::_followable_type arg)
  {
    msg_.followable = std::move(arg);
    return std::move(msg_);
  }

private:
  ::navigation_interface::msg::Obstacle msg_;
};

class Init_Obstacle_radius
{
public:
  explicit Init_Obstacle_radius(::navigation_interface::msg::Obstacle & msg)
  : msg_(msg)
  {}
  Init_Obstacle_followable radius(::navigation_interface::msg::Obstacle::_radius_type arg)
  {
    msg_.radius = std::move(arg);
    return Init_Obstacle_followable(msg_);
  }

private:
  ::navigation_interface::msg::Obstacle msg_;
};

class Init_Obstacle_pos
{
public:
  explicit Init_Obstacle_pos(::navigation_interface::msg::Obstacle & msg)
  : msg_(msg)
  {}
  Init_Obstacle_radius pos(::navigation_interface::msg::Obstacle::_pos_type arg)
  {
    msg_.pos = std::move(arg);
    return Init_Obstacle_radius(msg_);
  }

private:
  ::navigation_interface::msg::Obstacle msg_;
};

class Init_Obstacle_header
{
public:
  Init_Obstacle_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Obstacle_pos header(::navigation_interface::msg::Obstacle::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Obstacle_pos(msg_);
  }

private:
  ::navigation_interface::msg::Obstacle msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::navigation_interface::msg::Obstacle>()
{
  return navigation_interface::msg::builder::Init_Obstacle_header();
}

}  // namespace navigation_interface

#endif  // NAVIGATION_INTERFACE__MSG__DETAIL__OBSTACLE__BUILDER_HPP_
