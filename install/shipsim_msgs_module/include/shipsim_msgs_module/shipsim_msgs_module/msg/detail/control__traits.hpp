// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from shipsim_msgs_module:msg/Control.idl
// generated code does not contain a copyright notice

#ifndef SHIPSIM_MSGS_MODULE__MSG__DETAIL__CONTROL__TRAITS_HPP_
#define SHIPSIM_MSGS_MODULE__MSG__DETAIL__CONTROL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "shipsim_msgs_module/msg/detail/control__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace shipsim_msgs_module
{

namespace msg
{

inline void to_flow_style_yaml(
  const Control & msg,
  std::ostream & out)
{
  out << "{";
  // member: n_p
  {
    out << "n_p: ";
    rosidl_generator_traits::value_to_yaml(msg.n_p, out);
    out << ", ";
  }

  // member: rudder_angle_degree
  {
    out << "rudder_angle_degree: ";
    rosidl_generator_traits::value_to_yaml(msg.rudder_angle_degree, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Control & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: n_p
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "n_p: ";
    rosidl_generator_traits::value_to_yaml(msg.n_p, out);
    out << "\n";
  }

  // member: rudder_angle_degree
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rudder_angle_degree: ";
    rosidl_generator_traits::value_to_yaml(msg.rudder_angle_degree, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Control & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace shipsim_msgs_module

namespace rosidl_generator_traits
{

[[deprecated("use shipsim_msgs_module::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const shipsim_msgs_module::msg::Control & msg,
  std::ostream & out, size_t indentation = 0)
{
  shipsim_msgs_module::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use shipsim_msgs_module::msg::to_yaml() instead")]]
inline std::string to_yaml(const shipsim_msgs_module::msg::Control & msg)
{
  return shipsim_msgs_module::msg::to_yaml(msg);
}

template<>
inline const char * data_type<shipsim_msgs_module::msg::Control>()
{
  return "shipsim_msgs_module::msg::Control";
}

template<>
inline const char * name<shipsim_msgs_module::msg::Control>()
{
  return "shipsim_msgs_module/msg/Control";
}

template<>
struct has_fixed_size<shipsim_msgs_module::msg::Control>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<shipsim_msgs_module::msg::Control>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<shipsim_msgs_module::msg::Control>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SHIPSIM_MSGS_MODULE__MSG__DETAIL__CONTROL__TRAITS_HPP_
