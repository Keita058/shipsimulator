// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from shipsim_msgs_module:msg/Control.idl
// generated code does not contain a copyright notice
#include "shipsim_msgs_module/msg/detail/control__rosidl_typesupport_fastrtps_cpp.hpp"
#include "shipsim_msgs_module/msg/detail/control__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace shipsim_msgs_module
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_shipsim_msgs_module
cdr_serialize(
  const shipsim_msgs_module::msg::Control & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: n_p
  cdr << ros_message.n_p;
  // Member: rudder_angle_degree
  cdr << ros_message.rudder_angle_degree;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_shipsim_msgs_module
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  shipsim_msgs_module::msg::Control & ros_message)
{
  // Member: n_p
  cdr >> ros_message.n_p;

  // Member: rudder_angle_degree
  cdr >> ros_message.rudder_angle_degree;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_shipsim_msgs_module
get_serialized_size(
  const shipsim_msgs_module::msg::Control & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: n_p
  {
    size_t item_size = sizeof(ros_message.n_p);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: rudder_angle_degree
  {
    size_t item_size = sizeof(ros_message.rudder_angle_degree);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_shipsim_msgs_module
max_serialized_size_Control(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: n_p
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: rudder_angle_degree
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  return current_alignment - initial_alignment;
}

static bool _Control__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const shipsim_msgs_module::msg::Control *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Control__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<shipsim_msgs_module::msg::Control *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Control__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const shipsim_msgs_module::msg::Control *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Control__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_Control(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _Control__callbacks = {
  "shipsim_msgs_module::msg",
  "Control",
  _Control__cdr_serialize,
  _Control__cdr_deserialize,
  _Control__get_serialized_size,
  _Control__max_serialized_size
};

static rosidl_message_type_support_t _Control__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Control__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace shipsim_msgs_module

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_shipsim_msgs_module
const rosidl_message_type_support_t *
get_message_type_support_handle<shipsim_msgs_module::msg::Control>()
{
  return &shipsim_msgs_module::msg::typesupport_fastrtps_cpp::_Control__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, shipsim_msgs_module, msg, Control)() {
  return &shipsim_msgs_module::msg::typesupport_fastrtps_cpp::_Control__handle;
}

#ifdef __cplusplus
}
#endif
