// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from arussim_msgs:msg/Trajectory.idl
// generated code does not contain a copyright notice

#ifndef ARUSSIM_MSGS__MSG__DETAIL__TRAJECTORY__STRUCT_HPP_
#define ARUSSIM_MSGS__MSG__DETAIL__TRAJECTORY__STRUCT_HPP_

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
// Member 'points'
#include "arussim_msgs/msg/detail/point_xy__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__arussim_msgs__msg__Trajectory __attribute__((deprecated))
#else
# define DEPRECATED__arussim_msgs__msg__Trajectory __declspec(deprecated)
#endif

namespace arussim_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Trajectory_
{
  using Type = Trajectory_<ContainerAllocator>;

  explicit Trajectory_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit Trajectory_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _points_type =
    std::vector<arussim_msgs::msg::PointXY_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<arussim_msgs::msg::PointXY_<ContainerAllocator>>>;
  _points_type points;
  using _s_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _s_type s;
  using _k_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _k_type k;
  using _speed_profile_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _speed_profile_type speed_profile;
  using _acc_profile_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _acc_profile_type acc_profile;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__points(
    const std::vector<arussim_msgs::msg::PointXY_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<arussim_msgs::msg::PointXY_<ContainerAllocator>>> & _arg)
  {
    this->points = _arg;
    return *this;
  }
  Type & set__s(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->s = _arg;
    return *this;
  }
  Type & set__k(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->k = _arg;
    return *this;
  }
  Type & set__speed_profile(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->speed_profile = _arg;
    return *this;
  }
  Type & set__acc_profile(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->acc_profile = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    arussim_msgs::msg::Trajectory_<ContainerAllocator> *;
  using ConstRawPtr =
    const arussim_msgs::msg::Trajectory_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<arussim_msgs::msg::Trajectory_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<arussim_msgs::msg::Trajectory_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      arussim_msgs::msg::Trajectory_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<arussim_msgs::msg::Trajectory_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      arussim_msgs::msg::Trajectory_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<arussim_msgs::msg::Trajectory_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<arussim_msgs::msg::Trajectory_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<arussim_msgs::msg::Trajectory_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__arussim_msgs__msg__Trajectory
    std::shared_ptr<arussim_msgs::msg::Trajectory_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__arussim_msgs__msg__Trajectory
    std::shared_ptr<arussim_msgs::msg::Trajectory_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Trajectory_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->points != other.points) {
      return false;
    }
    if (this->s != other.s) {
      return false;
    }
    if (this->k != other.k) {
      return false;
    }
    if (this->speed_profile != other.speed_profile) {
      return false;
    }
    if (this->acc_profile != other.acc_profile) {
      return false;
    }
    return true;
  }
  bool operator!=(const Trajectory_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Trajectory_

// alias to use template instance with default allocator
using Trajectory =
  arussim_msgs::msg::Trajectory_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace arussim_msgs

#endif  // ARUSSIM_MSGS__MSG__DETAIL__TRAJECTORY__STRUCT_HPP_
