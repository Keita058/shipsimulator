// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from shipsim_msgs_module:msg/Control.idl
// generated code does not contain a copyright notice

#ifndef SHIPSIM_MSGS_MODULE__MSG__DETAIL__CONTROL__BUILDER_HPP_
#define SHIPSIM_MSGS_MODULE__MSG__DETAIL__CONTROL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "shipsim_msgs_module/msg/detail/control__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace shipsim_msgs_module
{

namespace msg
{

namespace builder
{

class Init_Control_rudder_angle_degree
{
public:
  explicit Init_Control_rudder_angle_degree(::shipsim_msgs_module::msg::Control & msg)
  : msg_(msg)
  {}
  ::shipsim_msgs_module::msg::Control rudder_angle_degree(::shipsim_msgs_module::msg::Control::_rudder_angle_degree_type arg)
  {
    msg_.rudder_angle_degree = std::move(arg);
    return std::move(msg_);
  }

private:
  ::shipsim_msgs_module::msg::Control msg_;
};

class Init_Control_n_p
{
public:
  Init_Control_n_p()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Control_rudder_angle_degree n_p(::shipsim_msgs_module::msg::Control::_n_p_type arg)
  {
    msg_.n_p = std::move(arg);
    return Init_Control_rudder_angle_degree(msg_);
  }

private:
  ::shipsim_msgs_module::msg::Control msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::shipsim_msgs_module::msg::Control>()
{
  return shipsim_msgs_module::msg::builder::Init_Control_n_p();
}

}  // namespace shipsim_msgs_module

#endif  // SHIPSIM_MSGS_MODULE__MSG__DETAIL__CONTROL__BUILDER_HPP_
