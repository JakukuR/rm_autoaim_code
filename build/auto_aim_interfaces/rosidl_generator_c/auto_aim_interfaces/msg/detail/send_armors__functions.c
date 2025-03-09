// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from auto_aim_interfaces:msg/SendArmors.idl
// generated code does not contain a copyright notice
#include "auto_aim_interfaces/msg/detail/send_armors__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
auto_aim_interfaces__msg__SendArmors__init(auto_aim_interfaces__msg__SendArmors * msg)
{
  if (!msg) {
    return false;
  }
  // yaw_angle
  // pitch_angle
  // distance
  // shoot_flag
  // detect_flag
  return true;
}

void
auto_aim_interfaces__msg__SendArmors__fini(auto_aim_interfaces__msg__SendArmors * msg)
{
  if (!msg) {
    return;
  }
  // yaw_angle
  // pitch_angle
  // distance
  // shoot_flag
  // detect_flag
}

bool
auto_aim_interfaces__msg__SendArmors__are_equal(const auto_aim_interfaces__msg__SendArmors * lhs, const auto_aim_interfaces__msg__SendArmors * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // yaw_angle
  if (lhs->yaw_angle != rhs->yaw_angle) {
    return false;
  }
  // pitch_angle
  if (lhs->pitch_angle != rhs->pitch_angle) {
    return false;
  }
  // distance
  if (lhs->distance != rhs->distance) {
    return false;
  }
  // shoot_flag
  if (lhs->shoot_flag != rhs->shoot_flag) {
    return false;
  }
  // detect_flag
  if (lhs->detect_flag != rhs->detect_flag) {
    return false;
  }
  return true;
}

bool
auto_aim_interfaces__msg__SendArmors__copy(
  const auto_aim_interfaces__msg__SendArmors * input,
  auto_aim_interfaces__msg__SendArmors * output)
{
  if (!input || !output) {
    return false;
  }
  // yaw_angle
  output->yaw_angle = input->yaw_angle;
  // pitch_angle
  output->pitch_angle = input->pitch_angle;
  // distance
  output->distance = input->distance;
  // shoot_flag
  output->shoot_flag = input->shoot_flag;
  // detect_flag
  output->detect_flag = input->detect_flag;
  return true;
}

auto_aim_interfaces__msg__SendArmors *
auto_aim_interfaces__msg__SendArmors__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto_aim_interfaces__msg__SendArmors * msg = (auto_aim_interfaces__msg__SendArmors *)allocator.allocate(sizeof(auto_aim_interfaces__msg__SendArmors), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(auto_aim_interfaces__msg__SendArmors));
  bool success = auto_aim_interfaces__msg__SendArmors__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
auto_aim_interfaces__msg__SendArmors__destroy(auto_aim_interfaces__msg__SendArmors * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    auto_aim_interfaces__msg__SendArmors__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
auto_aim_interfaces__msg__SendArmors__Sequence__init(auto_aim_interfaces__msg__SendArmors__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto_aim_interfaces__msg__SendArmors * data = NULL;

  if (size) {
    data = (auto_aim_interfaces__msg__SendArmors *)allocator.zero_allocate(size, sizeof(auto_aim_interfaces__msg__SendArmors), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = auto_aim_interfaces__msg__SendArmors__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        auto_aim_interfaces__msg__SendArmors__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
auto_aim_interfaces__msg__SendArmors__Sequence__fini(auto_aim_interfaces__msg__SendArmors__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      auto_aim_interfaces__msg__SendArmors__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

auto_aim_interfaces__msg__SendArmors__Sequence *
auto_aim_interfaces__msg__SendArmors__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto_aim_interfaces__msg__SendArmors__Sequence * array = (auto_aim_interfaces__msg__SendArmors__Sequence *)allocator.allocate(sizeof(auto_aim_interfaces__msg__SendArmors__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = auto_aim_interfaces__msg__SendArmors__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
auto_aim_interfaces__msg__SendArmors__Sequence__destroy(auto_aim_interfaces__msg__SendArmors__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    auto_aim_interfaces__msg__SendArmors__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
auto_aim_interfaces__msg__SendArmors__Sequence__are_equal(const auto_aim_interfaces__msg__SendArmors__Sequence * lhs, const auto_aim_interfaces__msg__SendArmors__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!auto_aim_interfaces__msg__SendArmors__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
auto_aim_interfaces__msg__SendArmors__Sequence__copy(
  const auto_aim_interfaces__msg__SendArmors__Sequence * input,
  auto_aim_interfaces__msg__SendArmors__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(auto_aim_interfaces__msg__SendArmors);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    auto_aim_interfaces__msg__SendArmors * data =
      (auto_aim_interfaces__msg__SendArmors *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!auto_aim_interfaces__msg__SendArmors__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          auto_aim_interfaces__msg__SendArmors__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!auto_aim_interfaces__msg__SendArmors__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
