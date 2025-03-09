// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from auto_aim_interfaces:msg/SendArmors.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__SEND_ARMORS__BUILDER_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__SEND_ARMORS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "auto_aim_interfaces/msg/detail/send_armors__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace auto_aim_interfaces
{

namespace msg
{

namespace builder
{

class Init_SendArmors_detect_flag
{
public:
  explicit Init_SendArmors_detect_flag(::auto_aim_interfaces::msg::SendArmors & msg)
  : msg_(msg)
  {}
  ::auto_aim_interfaces::msg::SendArmors detect_flag(::auto_aim_interfaces::msg::SendArmors::_detect_flag_type arg)
  {
    msg_.detect_flag = std::move(arg);
    return std::move(msg_);
  }

private:
  ::auto_aim_interfaces::msg::SendArmors msg_;
};

class Init_SendArmors_shoot_flag
{
public:
  explicit Init_SendArmors_shoot_flag(::auto_aim_interfaces::msg::SendArmors & msg)
  : msg_(msg)
  {}
  Init_SendArmors_detect_flag shoot_flag(::auto_aim_interfaces::msg::SendArmors::_shoot_flag_type arg)
  {
    msg_.shoot_flag = std::move(arg);
    return Init_SendArmors_detect_flag(msg_);
  }

private:
  ::auto_aim_interfaces::msg::SendArmors msg_;
};

class Init_SendArmors_distance
{
public:
  explicit Init_SendArmors_distance(::auto_aim_interfaces::msg::SendArmors & msg)
  : msg_(msg)
  {}
  Init_SendArmors_shoot_flag distance(::auto_aim_interfaces::msg::SendArmors::_distance_type arg)
  {
    msg_.distance = std::move(arg);
    return Init_SendArmors_shoot_flag(msg_);
  }

private:
  ::auto_aim_interfaces::msg::SendArmors msg_;
};

class Init_SendArmors_pitch_angle
{
public:
  explicit Init_SendArmors_pitch_angle(::auto_aim_interfaces::msg::SendArmors & msg)
  : msg_(msg)
  {}
  Init_SendArmors_distance pitch_angle(::auto_aim_interfaces::msg::SendArmors::_pitch_angle_type arg)
  {
    msg_.pitch_angle = std::move(arg);
    return Init_SendArmors_distance(msg_);
  }

private:
  ::auto_aim_interfaces::msg::SendArmors msg_;
};

class Init_SendArmors_yaw_angle
{
public:
  Init_SendArmors_yaw_angle()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SendArmors_pitch_angle yaw_angle(::auto_aim_interfaces::msg::SendArmors::_yaw_angle_type arg)
  {
    msg_.yaw_angle = std::move(arg);
    return Init_SendArmors_pitch_angle(msg_);
  }

private:
  ::auto_aim_interfaces::msg::SendArmors msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::auto_aim_interfaces::msg::SendArmors>()
{
  return auto_aim_interfaces::msg::builder::Init_SendArmors_yaw_angle();
}

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__SEND_ARMORS__BUILDER_HPP_
