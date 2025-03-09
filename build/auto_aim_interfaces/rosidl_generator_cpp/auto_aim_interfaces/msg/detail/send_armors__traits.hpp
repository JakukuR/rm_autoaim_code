// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from auto_aim_interfaces:msg/SendArmors.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__SEND_ARMORS__TRAITS_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__SEND_ARMORS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "auto_aim_interfaces/msg/detail/send_armors__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace auto_aim_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const SendArmors & msg,
  std::ostream & out)
{
  out << "{";
  // member: yaw_angle
  {
    out << "yaw_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_angle, out);
    out << ", ";
  }

  // member: pitch_angle
  {
    out << "pitch_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_angle, out);
    out << ", ";
  }

  // member: distance
  {
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
    out << ", ";
  }

  // member: shoot_flag
  {
    out << "shoot_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.shoot_flag, out);
    out << ", ";
  }

  // member: detect_flag
  {
    out << "detect_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.detect_flag, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SendArmors & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: yaw_angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_angle, out);
    out << "\n";
  }

  // member: pitch_angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_angle, out);
    out << "\n";
  }

  // member: distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
    out << "\n";
  }

  // member: shoot_flag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "shoot_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.shoot_flag, out);
    out << "\n";
  }

  // member: detect_flag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "detect_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.detect_flag, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SendArmors & msg, bool use_flow_style = false)
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

}  // namespace auto_aim_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use auto_aim_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const auto_aim_interfaces::msg::SendArmors & msg,
  std::ostream & out, size_t indentation = 0)
{
  auto_aim_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use auto_aim_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const auto_aim_interfaces::msg::SendArmors & msg)
{
  return auto_aim_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<auto_aim_interfaces::msg::SendArmors>()
{
  return "auto_aim_interfaces::msg::SendArmors";
}

template<>
inline const char * name<auto_aim_interfaces::msg::SendArmors>()
{
  return "auto_aim_interfaces/msg/SendArmors";
}

template<>
struct has_fixed_size<auto_aim_interfaces::msg::SendArmors>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<auto_aim_interfaces::msg::SendArmors>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<auto_aim_interfaces::msg::SendArmors>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__SEND_ARMORS__TRAITS_HPP_
