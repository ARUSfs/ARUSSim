// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from arussim_msgs:msg/PointXY.idl
// generated code does not contain a copyright notice

#ifndef ARUSSIM_MSGS__MSG__DETAIL__POINT_XY__STRUCT_HPP_
#define ARUSSIM_MSGS__MSG__DETAIL__POINT_XY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__arussim_msgs__msg__PointXY __attribute__((deprecated))
#else
# define DEPRECATED__arussim_msgs__msg__PointXY __declspec(deprecated)
#endif

namespace arussim_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PointXY_
{
  using Type = PointXY_<ContainerAllocator>;

  explicit PointXY_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0f;
      this->y = 0.0f;
    }
  }

  explicit PointXY_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0f;
      this->y = 0.0f;
    }
  }

  // field types and members
  using _x_type =
    float;
  _x_type x;
  using _y_type =
    float;
  _y_type y;

  // setters for named parameter idiom
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

  // constant declarations

  // pointer types
  using RawPtr =
    arussim_msgs::msg::PointXY_<ContainerAllocator> *;
  using ConstRawPtr =
    const arussim_msgs::msg::PointXY_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<arussim_msgs::msg::PointXY_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<arussim_msgs::msg::PointXY_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      arussim_msgs::msg::PointXY_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<arussim_msgs::msg::PointXY_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      arussim_msgs::msg::PointXY_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<arussim_msgs::msg::PointXY_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<arussim_msgs::msg::PointXY_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<arussim_msgs::msg::PointXY_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__arussim_msgs__msg__PointXY
    std::shared_ptr<arussim_msgs::msg::PointXY_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__arussim_msgs__msg__PointXY
    std::shared_ptr<arussim_msgs::msg::PointXY_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PointXY_ & other) const
  {
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    return true;
  }
  bool operator!=(const PointXY_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PointXY_

// alias to use template instance with default allocator
using PointXY =
  arussim_msgs::msg::PointXY_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace arussim_msgs

#endif  // ARUSSIM_MSGS__MSG__DETAIL__POINT_XY__STRUCT_HPP_
