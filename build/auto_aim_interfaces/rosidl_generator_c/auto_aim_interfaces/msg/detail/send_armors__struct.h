// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from auto_aim_interfaces:msg/SendArmors.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__SEND_ARMORS__STRUCT_H_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__SEND_ARMORS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/SendArmors in the package auto_aim_interfaces.
typedef struct auto_aim_interfaces__msg__SendArmors
{
  double yaw_angle;
  double pitch_angle;
  double distance;
  int32_t shoot_flag;
  int32_t detect_flag;
} auto_aim_interfaces__msg__SendArmors;

// Struct for a sequence of auto_aim_interfaces__msg__SendArmors.
typedef struct auto_aim_interfaces__msg__SendArmors__Sequence
{
  auto_aim_interfaces__msg__SendArmors * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} auto_aim_interfaces__msg__SendArmors__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__SEND_ARMORS__STRUCT_H_
