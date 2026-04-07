// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from navigation_interface:msg/Stop.idl
// generated code does not contain a copyright notice
#include "navigation_interface/msg/detail/stop__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `sender_id`
#include "std_msgs/msg/detail/string__functions.h"

bool
navigation_interface__msg__Stop__init(navigation_interface__msg__Stop * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    navigation_interface__msg__Stop__fini(msg);
    return false;
  }
  // stop
  // sender_id
  if (!std_msgs__msg__String__init(&msg->sender_id)) {
    navigation_interface__msg__Stop__fini(msg);
    return false;
  }
  // distance
  return true;
}

void
navigation_interface__msg__Stop__fini(navigation_interface__msg__Stop * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // stop
  // sender_id
  std_msgs__msg__String__fini(&msg->sender_id);
  // distance
}

bool
navigation_interface__msg__Stop__are_equal(const navigation_interface__msg__Stop * lhs, const navigation_interface__msg__Stop * rhs)
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
  // stop
  if (lhs->stop != rhs->stop) {
    return false;
  }
  // sender_id
  if (!std_msgs__msg__String__are_equal(
      &(lhs->sender_id), &(rhs->sender_id)))
  {
    return false;
  }
  // distance
  if (lhs->distance != rhs->distance) {
    return false;
  }
  return true;
}

bool
navigation_interface__msg__Stop__copy(
  const navigation_interface__msg__Stop * input,
  navigation_interface__msg__Stop * output)
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
  // stop
  output->stop = input->stop;
  // sender_id
  if (!std_msgs__msg__String__copy(
      &(input->sender_id), &(output->sender_id)))
  {
    return false;
  }
  // distance
  output->distance = input->distance;
  return true;
}

navigation_interface__msg__Stop *
navigation_interface__msg__Stop__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  navigation_interface__msg__Stop * msg = (navigation_interface__msg__Stop *)allocator.allocate(sizeof(navigation_interface__msg__Stop), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(navigation_interface__msg__Stop));
  bool success = navigation_interface__msg__Stop__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
navigation_interface__msg__Stop__destroy(navigation_interface__msg__Stop * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    navigation_interface__msg__Stop__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
navigation_interface__msg__Stop__Sequence__init(navigation_interface__msg__Stop__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  navigation_interface__msg__Stop * data = NULL;

  if (size) {
    data = (navigation_interface__msg__Stop *)allocator.zero_allocate(size, sizeof(navigation_interface__msg__Stop), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = navigation_interface__msg__Stop__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        navigation_interface__msg__Stop__fini(&data[i - 1]);
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
navigation_interface__msg__Stop__Sequence__fini(navigation_interface__msg__Stop__Sequence * array)
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
      navigation_interface__msg__Stop__fini(&array->data[i]);
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

navigation_interface__msg__Stop__Sequence *
navigation_interface__msg__Stop__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  navigation_interface__msg__Stop__Sequence * array = (navigation_interface__msg__Stop__Sequence *)allocator.allocate(sizeof(navigation_interface__msg__Stop__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = navigation_interface__msg__Stop__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
navigation_interface__msg__Stop__Sequence__destroy(navigation_interface__msg__Stop__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    navigation_interface__msg__Stop__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
navigation_interface__msg__Stop__Sequence__are_equal(const navigation_interface__msg__Stop__Sequence * lhs, const navigation_interface__msg__Stop__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!navigation_interface__msg__Stop__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
navigation_interface__msg__Stop__Sequence__copy(
  const navigation_interface__msg__Stop__Sequence * input,
  navigation_interface__msg__Stop__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(navigation_interface__msg__Stop);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    navigation_interface__msg__Stop * data =
      (navigation_interface__msg__Stop *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!navigation_interface__msg__Stop__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          navigation_interface__msg__Stop__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!navigation_interface__msg__Stop__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
