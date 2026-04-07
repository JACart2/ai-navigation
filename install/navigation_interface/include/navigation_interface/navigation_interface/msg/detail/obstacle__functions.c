// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from navigation_interface:msg/Obstacle.idl
// generated code does not contain a copyright notice
#include "navigation_interface/msg/detail/obstacle__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `pos`
#include "geometry_msgs/msg/detail/point_stamped__functions.h"

bool
navigation_interface__msg__Obstacle__init(navigation_interface__msg__Obstacle * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    navigation_interface__msg__Obstacle__fini(msg);
    return false;
  }
  // pos
  if (!geometry_msgs__msg__PointStamped__init(&msg->pos)) {
    navigation_interface__msg__Obstacle__fini(msg);
    return false;
  }
  // radius
  // followable
  return true;
}

void
navigation_interface__msg__Obstacle__fini(navigation_interface__msg__Obstacle * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // pos
  geometry_msgs__msg__PointStamped__fini(&msg->pos);
  // radius
  // followable
}

bool
navigation_interface__msg__Obstacle__are_equal(const navigation_interface__msg__Obstacle * lhs, const navigation_interface__msg__Obstacle * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // pos
  if (!geometry_msgs__msg__PointStamped__are_equal(
      &(lhs->pos), &(rhs->pos)))
  {
    return false;
  }
  // radius
  if (lhs->radius != rhs->radius) {
    return false;
  }
  // followable
  if (lhs->followable != rhs->followable) {
    return false;
  }
  return true;
}

bool
navigation_interface__msg__Obstacle__copy(
  const navigation_interface__msg__Obstacle * input,
  navigation_interface__msg__Obstacle * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // pos
  if (!geometry_msgs__msg__PointStamped__copy(
      &(input->pos), &(output->pos)))
  {
    return false;
  }
  // radius
  output->radius = input->radius;
  // followable
  output->followable = input->followable;
  return true;
}

navigation_interface__msg__Obstacle *
navigation_interface__msg__Obstacle__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  navigation_interface__msg__Obstacle * msg = (navigation_interface__msg__Obstacle *)allocator.allocate(sizeof(navigation_interface__msg__Obstacle), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(navigation_interface__msg__Obstacle));
  bool success = navigation_interface__msg__Obstacle__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
navigation_interface__msg__Obstacle__destroy(navigation_interface__msg__Obstacle * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    navigation_interface__msg__Obstacle__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
navigation_interface__msg__Obstacle__Sequence__init(navigation_interface__msg__Obstacle__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  navigation_interface__msg__Obstacle * data = NULL;

  if (size) {
    data = (navigation_interface__msg__Obstacle *)allocator.zero_allocate(size, sizeof(navigation_interface__msg__Obstacle), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = navigation_interface__msg__Obstacle__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        navigation_interface__msg__Obstacle__fini(&data[i - 1]);
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
navigation_interface__msg__Obstacle__Sequence__fini(navigation_interface__msg__Obstacle__Sequence * array)
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
      navigation_interface__msg__Obstacle__fini(&array->data[i]);
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

navigation_interface__msg__Obstacle__Sequence *
navigation_interface__msg__Obstacle__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  navigation_interface__msg__Obstacle__Sequence * array = (navigation_interface__msg__Obstacle__Sequence *)allocator.allocate(sizeof(navigation_interface__msg__Obstacle__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = navigation_interface__msg__Obstacle__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
navigation_interface__msg__Obstacle__Sequence__destroy(navigation_interface__msg__Obstacle__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    navigation_interface__msg__Obstacle__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
navigation_interface__msg__Obstacle__Sequence__are_equal(const navigation_interface__msg__Obstacle__Sequence * lhs, const navigation_interface__msg__Obstacle__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!navigation_interface__msg__Obstacle__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
navigation_interface__msg__Obstacle__Sequence__copy(
  const navigation_interface__msg__Obstacle__Sequence * input,
  navigation_interface__msg__Obstacle__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(navigation_interface__msg__Obstacle);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    navigation_interface__msg__Obstacle * data =
      (navigation_interface__msg__Obstacle *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!navigation_interface__msg__Obstacle__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          navigation_interface__msg__Obstacle__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!navigation_interface__msg__Obstacle__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
