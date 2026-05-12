// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from navigation_interface:msg/LatLongArray.idl
// generated code does not contain a copyright notice
#include "navigation_interface/msg/detail/lat_long_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `gpspoints`
#include "navigation_interface/msg/detail/lat_long_point__functions.h"

bool
navigation_interface__msg__LatLongArray__init(navigation_interface__msg__LatLongArray * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    navigation_interface__msg__LatLongArray__fini(msg);
    return false;
  }
  // gpspoints
  if (!navigation_interface__msg__LatLongPoint__Sequence__init(&msg->gpspoints, 0)) {
    navigation_interface__msg__LatLongArray__fini(msg);
    return false;
  }
  return true;
}

void
navigation_interface__msg__LatLongArray__fini(navigation_interface__msg__LatLongArray * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // gpspoints
  navigation_interface__msg__LatLongPoint__Sequence__fini(&msg->gpspoints);
}

bool
navigation_interface__msg__LatLongArray__are_equal(const navigation_interface__msg__LatLongArray * lhs, const navigation_interface__msg__LatLongArray * rhs)
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
  // gpspoints
  if (!navigation_interface__msg__LatLongPoint__Sequence__are_equal(
      &(lhs->gpspoints), &(rhs->gpspoints)))
  {
    return false;
  }
  return true;
}

bool
navigation_interface__msg__LatLongArray__copy(
  const navigation_interface__msg__LatLongArray * input,
  navigation_interface__msg__LatLongArray * output)
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
  // gpspoints
  if (!navigation_interface__msg__LatLongPoint__Sequence__copy(
      &(input->gpspoints), &(output->gpspoints)))
  {
    return false;
  }
  return true;
}

navigation_interface__msg__LatLongArray *
navigation_interface__msg__LatLongArray__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  navigation_interface__msg__LatLongArray * msg = (navigation_interface__msg__LatLongArray *)allocator.allocate(sizeof(navigation_interface__msg__LatLongArray), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(navigation_interface__msg__LatLongArray));
  bool success = navigation_interface__msg__LatLongArray__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
navigation_interface__msg__LatLongArray__destroy(navigation_interface__msg__LatLongArray * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    navigation_interface__msg__LatLongArray__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
navigation_interface__msg__LatLongArray__Sequence__init(navigation_interface__msg__LatLongArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  navigation_interface__msg__LatLongArray * data = NULL;

  if (size) {
    data = (navigation_interface__msg__LatLongArray *)allocator.zero_allocate(size, sizeof(navigation_interface__msg__LatLongArray), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = navigation_interface__msg__LatLongArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        navigation_interface__msg__LatLongArray__fini(&data[i - 1]);
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
navigation_interface__msg__LatLongArray__Sequence__fini(navigation_interface__msg__LatLongArray__Sequence * array)
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
      navigation_interface__msg__LatLongArray__fini(&array->data[i]);
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

navigation_interface__msg__LatLongArray__Sequence *
navigation_interface__msg__LatLongArray__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  navigation_interface__msg__LatLongArray__Sequence * array = (navigation_interface__msg__LatLongArray__Sequence *)allocator.allocate(sizeof(navigation_interface__msg__LatLongArray__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = navigation_interface__msg__LatLongArray__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
navigation_interface__msg__LatLongArray__Sequence__destroy(navigation_interface__msg__LatLongArray__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    navigation_interface__msg__LatLongArray__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
navigation_interface__msg__LatLongArray__Sequence__are_equal(const navigation_interface__msg__LatLongArray__Sequence * lhs, const navigation_interface__msg__LatLongArray__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!navigation_interface__msg__LatLongArray__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
navigation_interface__msg__LatLongArray__Sequence__copy(
  const navigation_interface__msg__LatLongArray__Sequence * input,
  navigation_interface__msg__LatLongArray__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(navigation_interface__msg__LatLongArray);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    navigation_interface__msg__LatLongArray * data =
      (navigation_interface__msg__LatLongArray *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!navigation_interface__msg__LatLongArray__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          navigation_interface__msg__LatLongArray__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!navigation_interface__msg__LatLongArray__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
