// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from auto_aim_interfaces:msg/SendArmors.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__SEND_ARMORS__STRUCT_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__SEND_ARMORS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__auto_aim_interfaces__msg__SendArmors __attribute__((deprecated))
#else
# define DEPRECATED__auto_aim_interfaces__msg__SendArmors __declspec(deprecated)
#endif

namespace auto_aim_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SendArmors_
{
  using Type = SendArmors_<ContainerAllocator>;

  explicit SendArmors_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->yaw_angle = 0.0;
      this->pitch_angle = 0.0;
      this->distance = 0.0;
      this->shoot_flag = 0l;
      this->detect_flag = 0l;
    }
  }

  explicit SendArmors_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->yaw_angle = 0.0;
      this->pitch_angle = 0.0;
      this->distance = 0.0;
      this->shoot_flag = 0l;
      this->detect_flag = 0l;
    }
  }

  // field types and members
  using _yaw_angle_type =
    double;
  _yaw_angle_type yaw_angle;
  using _pitch_angle_type =
    double;
  _pitch_angle_type pitch_angle;
  using _distance_type =
    double;
  _distance_type distance;
  using _shoot_flag_type =
    int32_t;
  _shoot_flag_type shoot_flag;
  using _detect_flag_type =
    int32_t;
  _detect_flag_type detect_flag;

  // setters for named parameter idiom
  Type & set__yaw_angle(
    const double & _arg)
  {
    this->yaw_angle = _arg;
    return *this;
  }
  Type & set__pitch_angle(
    const double & _arg)
  {
    this->pitch_angle = _arg;
    return *this;
  }
  Type & set__distance(
    const double & _arg)
  {
    this->distance = _arg;
    return *this;
  }
  Type & set__shoot_flag(
    const int32_t & _arg)
  {
    this->shoot_flag = _arg;
    return *this;
  }
  Type & set__detect_flag(
    const int32_t & _arg)
  {
    this->detect_flag = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    auto_aim_interfaces::msg::SendArmors_<ContainerAllocator> *;
  using ConstRawPtr =
    const auto_aim_interfaces::msg::SendArmors_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<auto_aim_interfaces::msg::SendArmors_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<auto_aim_interfaces::msg::SendArmors_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      auto_aim_interfaces::msg::SendArmors_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<auto_aim_interfaces::msg::SendArmors_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      auto_aim_interfaces::msg::SendArmors_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<auto_aim_interfaces::msg::SendArmors_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<auto_aim_interfaces::msg::SendArmors_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<auto_aim_interfaces::msg::SendArmors_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__auto_aim_interfaces__msg__SendArmors
    std::shared_ptr<auto_aim_interfaces::msg::SendArmors_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__auto_aim_interfaces__msg__SendArmors
    std::shared_ptr<auto_aim_interfaces::msg::SendArmors_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SendArmors_ & other) const
  {
    if (this->yaw_angle != other.yaw_angle) {
      return false;
    }
    if (this->pitch_angle != other.pitch_angle) {
      return false;
    }
    if (this->distance != other.distance) {
      return false;
    }
    if (this->shoot_flag != other.shoot_flag) {
      return false;
    }
    if (this->detect_flag != other.detect_flag) {
      return false;
    }
    return true;
  }
  bool operator!=(const SendArmors_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SendArmors_

// alias to use template instance with default allocator
using SendArmors =
  auto_aim_interfaces::msg::SendArmors_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__SEND_ARMORS__STRUCT_HPP_
