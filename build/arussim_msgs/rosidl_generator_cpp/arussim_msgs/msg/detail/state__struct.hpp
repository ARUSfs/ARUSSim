// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from arussim_msgs:msg/State.idl
// generated code does not contain a copyright notice

#ifndef ARUSSIM_MSGS__MSG__DETAIL__STATE__STRUCT_HPP_
#define ARUSSIM_MSGS__MSG__DETAIL__STATE__STRUCT_HPP_

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
// Member 'wheel_speeds'
// Member 'torque'
#include "arussim_msgs/msg/detail/four_wheel_drive__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__arussim_msgs__msg__State __attribute__((deprecated))
#else
# define DEPRECATED__arussim_msgs__msg__State __declspec(deprecated)
#endif

namespace arussim_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct State_
{
  using Type = State_<ContainerAllocator>;

  explicit State_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    wheel_speeds(_init),
    torque(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0f;
      this->y = 0.0f;
      this->yaw = 0.0f;
      this->vx = 0.0f;
      this->vy = 0.0f;
      this->r = 0.0f;
      this->ax = 0.0f;
      this->ay = 0.0f;
      this->delta = 0.0f;
    }
  }

  explicit State_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    wheel_speeds(_alloc, _init),
    torque(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0f;
      this->y = 0.0f;
      this->yaw = 0.0f;
      this->vx = 0.0f;
      this->vy = 0.0f;
      this->r = 0.0f;
      this->ax = 0.0f;
      this->ay = 0.0f;
      this->delta = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _x_type =
    float;
  _x_type x;
  using _y_type =
    float;
  _y_type y;
  using _yaw_type =
    float;
  _yaw_type yaw;
  using _vx_type =
    float;
  _vx_type vx;
  using _vy_type =
    float;
  _vy_type vy;
  using _r_type =
    float;
  _r_type r;
  using _ax_type =
    float;
  _ax_type ax;
  using _ay_type =
    float;
  _ay_type ay;
  using _delta_type =
    float;
  _delta_type delta;
  using _wheel_speeds_type =
    arussim_msgs::msg::FourWheelDrive_<ContainerAllocator>;
  _wheel_speeds_type wheel_speeds;
  using _torque_type =
    arussim_msgs::msg::FourWheelDrive_<ContainerAllocator>;
  _torque_type torque;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__x(
    const float & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const float & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__yaw(
    const float & _arg)
  {
    this->yaw = _arg;
    return *this;
  }
  Type & set__vx(
    const float & _arg)
  {
    this->vx = _arg;
    return *this;
  }
  Type & set__vy(
    const float & _arg)
  {
    this->vy = _arg;
    return *this;
  }
  Type & set__r(
    const float & _arg)
  {
    this->r = _arg;
    return *this;
  }
  Type & set__ax(
    const float & _arg)
  {
    this->ax = _arg;
    return *this;
  }
  Type & set__ay(
    const float & _arg)
  {
    this->ay = _arg;
    return *this;
  }
  Type & set__delta(
    const float & _arg)
  {
    this->delta = _arg;
    return *this;
  }
  Type & set__wheel_speeds(
    const arussim_msgs::msg::FourWheelDrive_<ContainerAllocator> & _arg)
  {
    this->wheel_speeds = _arg;
    return *this;
  }
  Type & set__torque(
    const arussim_msgs::msg::FourWheelDrive_<ContainerAllocator> & _arg)
  {
    this->torque = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    arussim_msgs::msg::State_<ContainerAllocator> *;
  using ConstRawPtr =
    const arussim_msgs::msg::State_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<arussim_msgs::msg::State_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<arussim_msgs::msg::State_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      arussim_msgs::msg::State_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<arussim_msgs::msg::State_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      arussim_msgs::msg::State_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<arussim_msgs::msg::State_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<arussim_msgs::msg::State_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<arussim_msgs::msg::State_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__arussim_msgs__msg__State
    std::shared_ptr<arussim_msgs::msg::State_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__arussim_msgs__msg__State
    std::shared_ptr<arussim_msgs::msg::State_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const State_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    if (this->vx != other.vx) {
      return false;
    }
    if (this->vy != other.vy) {
      return false;
    }
    if (this->r != other.r) {
      return false;
    }
    if (this->ax != other.ax) {
      return false;
    }
    if (this->ay != other.ay) {
      return false;
    }
    if (this->delta != other.delta) {
      return false;
    }
    if (this->wheel_speeds != other.wheel_speeds) {
      return false;
    }
    if (this->torque != other.torque) {
      return false;
    }
    return true;
  }
  bool operator!=(const State_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct State_

// alias to use template instance with default allocator
using State =
  arussim_msgs::msg::State_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace arussim_msgs

#endif  // ARUSSIM_MSGS__MSG__DETAIL__STATE__STRUCT_HPP_
