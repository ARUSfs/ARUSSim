// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from arussim_msgs:msg/Cmd4WD.idl
// generated code does not contain a copyright notice

#ifndef ARUSSIM_MSGS__MSG__DETAIL__CMD4_WD__STRUCT_HPP_
#define ARUSSIM_MSGS__MSG__DETAIL__CMD4_WD__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'acc'
#include "arussim_msgs/msg/detail/four_wheel_drive__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__arussim_msgs__msg__Cmd4WD __attribute__((deprecated))
#else
# define DEPRECATED__arussim_msgs__msg__Cmd4WD __declspec(deprecated)
#endif

namespace arussim_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Cmd4WD_
{
  using Type = Cmd4WD_<ContainerAllocator>;

  explicit Cmd4WD_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    acc(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->delta = 0.0f;
    }
  }

  explicit Cmd4WD_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    acc(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->delta = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _acc_type =
    arussim_msgs::msg::FourWheelDrive_<ContainerAllocator>;
  _acc_type acc;
  using _delta_type =
    float;
  _delta_type delta;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__acc(
    const arussim_msgs::msg::FourWheelDrive_<ContainerAllocator> & _arg)
  {
    this->acc = _arg;
    return *this;
  }
  Type & set__delta(
    const float & _arg)
  {
    this->delta = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    arussim_msgs::msg::Cmd4WD_<ContainerAllocator> *;
  using ConstRawPtr =
    const arussim_msgs::msg::Cmd4WD_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<arussim_msgs::msg::Cmd4WD_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<arussim_msgs::msg::Cmd4WD_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      arussim_msgs::msg::Cmd4WD_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<arussim_msgs::msg::Cmd4WD_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      arussim_msgs::msg::Cmd4WD_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<arussim_msgs::msg::Cmd4WD_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<arussim_msgs::msg::Cmd4WD_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<arussim_msgs::msg::Cmd4WD_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__arussim_msgs__msg__Cmd4WD
    std::shared_ptr<arussim_msgs::msg::Cmd4WD_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__arussim_msgs__msg__Cmd4WD
    std::shared_ptr<arussim_msgs::msg::Cmd4WD_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Cmd4WD_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->acc != other.acc) {
      return false;
    }
    if (this->delta != other.delta) {
      return false;
    }
    return true;
  }
  bool operator!=(const Cmd4WD_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Cmd4WD_

// alias to use template instance with default allocator
using Cmd4WD =
  arussim_msgs::msg::Cmd4WD_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace arussim_msgs

#endif  // ARUSSIM_MSGS__MSG__DETAIL__CMD4_WD__STRUCT_HPP_
