// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from navigation_interface:msg/LocalPointsArray.idl
// generated code does not contain a copyright notice
#include "navigation_interface/msg/detail/local_points_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `localpoints`
#include "geometry_msgs/msg/detail/pose__functions.h"
// Member `total_distance`
#include "std_msgs/msg/detail/float64__functions.h"

bool
navigation_interface__msg__LocalPointsArray__init(navigation_interface__msg__LocalPointsArray * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    navigation_interface__msg__LocalPointsArray__fini(msg);
    return false;
  }
  // localpoints
  if (!geometry_msgs__msg__Pose__Sequence__init(&msg->localpoints, 0)) {
    navigation_interface__msg__LocalPointsArray__fini(msg);
    return false;
  }
  // total_distance
  if (!std_msgs__msg__Float64__init(&msg->total_distance)) {
    navigation_interface__msg__LocalPointsArray__fini(msg);
    return false;
  }
  return true;
}

void
navigation_interface__msg__LocalPointsArray__fini(navigation_interface__msg__LocalPointsArray * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // localpoints
  geometry_msgs__msg__Pose__Sequence__fini(&msg->localpoints);
  // total_distance
  std_msgs__msg__Float64__fini(&msg->total_distance);
}

bool
navigation_interface__msg__LocalPointsArray__are_equal(const navigation_interface__msg__LocalPointsArray * lhs, const navigation_interface__msg__LocalPointsArray * rhs)
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
  // localpoints
  if (!geometry_msgs__msg__Pose__Sequence__are_equal(
      &(lhs->localpoints), &(rhs->localpoints)))
  {
    return false;
  }
  // total_distance
  if (!std_msgs__msg__Float64__are_equal(
      &(lhs->total_distance), &(rhs->total_distance)))
  {
    return false;
  }
  return true;
}

bool
navigation_interface__msg__LocalPointsArray__copy(
  const navigation_interface__msg__LocalPointsArray * input,
  navigation_interface__msg__LocalPointsArray * output)
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
  // localpoints
  if (!geometry_msgs__msg__Pose__Sequence__copy(
      &(input->localpoints), &(output->localpoints)))
  {
    return false;
  }
  // total_distance
  if (!std_msgs__msg__Float64__copy(
      &(input->total_distance), &(output->total_distance)))
  {
    return false;
  }
  return true;
}

navigation_interface__msg__LocalPointsArray *
navigation_interface__msg__LocalPointsArray__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  navigation_interface__msg__LocalPointsArray * msg = (navigation_interface__msg__LocalPointsArray *)allocator.allocate(sizeof(navigation_interface__msg__LocalPointsArray), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(navigation_interface__msg__LocalPointsArray));
  bool success = navigation_interface__msg__LocalPointsArray__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
navigation_interface__msg__LocalPointsArray__destroy(navigation_interface__msg__LocalPointsArray * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    navigation_interface__msg__LocalPointsArray__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
navigation_interface__msg__LocalPointsArray__Sequence__init(navigation_interface__msg__LocalPointsArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  navigation_interface__msg__LocalPointsArray * data = NULL;

  if (size) {
    data = (navigation_interface__msg__LocalPointsArray *)allocator.zero_allocate(size, sizeof(navigation_interface__msg__LocalPointsArray), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = navigation_interface__msg__LocalPointsArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        navigation_interface__msg__LocalPointsArray__fini(&data[i - 1]);
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
navigation_interface__msg__LocalPointsArray__Sequence__fini(navigation_interface__msg__LocalPointsArray__Sequence * array)
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
      navigation_interface__msg__LocalPointsArray__fini(&array->data[i]);
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

navigation_interface__msg__LocalPointsArray__Sequence *
navigation_interface__msg__LocalPointsArray__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  navigation_interface__msg__LocalPointsArray__Sequence * array = (navigation_interface__msg__LocalPointsArray__Sequence *)allocator.allocate(sizeof(navigation_interface__msg__LocalPointsArray__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = navigation_interface__msg__LocalPointsArray__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
navigation_interface__msg__LocalPointsArray__Sequence__destroy(navigation_interface__msg__LocalPointsArray__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    navigation_interface__msg__LocalPointsArray__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
navigation_interface__msg__LocalPointsArray__Sequence__are_equal(const navigation_interface__msg__LocalPointsArray__Sequence * lhs, const navigation_interface__msg__LocalPointsArray__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!navigation_interface__msg__LocalPointsArray__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
navigation_interface__msg__LocalPointsArray__Sequence__copy(
  const navigation_interface__msg__LocalPointsArray__Sequence * input,
  navigation_interface__msg__LocalPointsArray__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(navigation_interface__msg__LocalPointsArray);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    navigation_interface__msg__LocalPointsArray * data =
      (navigation_interface__msg__LocalPointsArray *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!navigation_interface__msg__LocalPointsArray__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          navigation_interface__msg__LocalPointsArray__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!navigation_interface__msg__LocalPointsArray__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
