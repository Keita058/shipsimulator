// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from shipsim_msgs_module:msg/Control.idl
// generated code does not contain a copyright notice

#ifndef SHIPSIM_MSGS_MODULE__MSG__DETAIL__CONTROL__STRUCT_HPP_
#define SHIPSIM_MSGS_MODULE__MSG__DETAIL__CONTROL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__shipsim_msgs_module__msg__Control __attribute__((deprecated))
#else
# define DEPRECATED__shipsim_msgs_module__msg__Control __declspec(deprecated)
#endif

namespace shipsim_msgs_module
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Control_
{
  using Type = Control_<ContainerAllocator>;

  explicit Control_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->n_p = 0.0;
      this->rudder_angle_degree = 0.0;
    }
  }

  explicit Control_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->n_p = 0.0;
      this->rudder_angle_degree = 0.0;
    }
  }

  // field types and members
  using _n_p_type =
    double;
  _n_p_type n_p;
  using _rudder_angle_degree_type =
    double;
  _rudder_angle_degree_type rudder_angle_degree;

  // setters for named parameter idiom
  Type & set__n_p(
    const double & _arg)
  {
    this->n_p = _arg;
    return *this;
  }
  Type & set__rudder_angle_degree(
    const double & _arg)
  {
    this->rudder_angle_degree = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    shipsim_msgs_module::msg::Control_<ContainerAllocator> *;
  using ConstRawPtr =
    const shipsim_msgs_module::msg::Control_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<shipsim_msgs_module::msg::Control_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<shipsim_msgs_module::msg::Control_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      shipsim_msgs_module::msg::Control_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<shipsim_msgs_module::msg::Control_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      shipsim_msgs_module::msg::Control_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<shipsim_msgs_module::msg::Control_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<shipsim_msgs_module::msg::Control_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<shipsim_msgs_module::msg::Control_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__shipsim_msgs_module__msg__Control
    std::shared_ptr<shipsim_msgs_module::msg::Control_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__shipsim_msgs_module__msg__Control
    std::shared_ptr<shipsim_msgs_module::msg::Control_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Control_ & other) const
  {
    if (this->n_p != other.n_p) {
      return false;
    }
    if (this->rudder_angle_degree != other.rudder_angle_degree) {
      return false;
    }
    return true;
  }
  bool operator!=(const Control_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Control_

// alias to use template instance with default allocator
using Control =
  shipsim_msgs_module::msg::Control_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace shipsim_msgs_module

#endif  // SHIPSIM_MSGS_MODULE__MSG__DETAIL__CONTROL__STRUCT_HPP_
