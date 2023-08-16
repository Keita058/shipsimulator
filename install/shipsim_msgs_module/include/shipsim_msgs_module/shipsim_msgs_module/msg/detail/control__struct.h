// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from shipsim_msgs_module:msg/Control.idl
// generated code does not contain a copyright notice

#ifndef SHIPSIM_MSGS_MODULE__MSG__DETAIL__CONTROL__STRUCT_H_
#define SHIPSIM_MSGS_MODULE__MSG__DETAIL__CONTROL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Control in the package shipsim_msgs_module.
typedef struct shipsim_msgs_module__msg__Control
{
  double n_p;
  double rudder_angle_degree;
} shipsim_msgs_module__msg__Control;

// Struct for a sequence of shipsim_msgs_module__msg__Control.
typedef struct shipsim_msgs_module__msg__Control__Sequence
{
  shipsim_msgs_module__msg__Control * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} shipsim_msgs_module__msg__Control__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SHIPSIM_MSGS_MODULE__MSG__DETAIL__CONTROL__STRUCT_H_
