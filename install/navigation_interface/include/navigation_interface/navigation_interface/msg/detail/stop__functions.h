// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from navigation_interface:msg/Stop.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "navigation_interface/msg/stop.h"


#ifndef NAVIGATION_INTERFACE__MSG__DETAIL__STOP__FUNCTIONS_H_
#define NAVIGATION_INTERFACE__MSG__DETAIL__STOP__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "navigation_interface/msg/rosidl_generator_c__visibility_control.h"

#include "navigation_interface/msg/detail/stop__struct.h"

/// Initialize msg/Stop message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * navigation_interface__msg__Stop
 * )) before or use
 * navigation_interface__msg__Stop__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_navigation_interface
bool
navigation_interface__msg__Stop__init(navigation_interface__msg__Stop * msg);

/// Finalize msg/Stop message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_navigation_interface
void
navigation_interface__msg__Stop__fini(navigation_interface__msg__Stop * msg);

/// Create msg/Stop message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * navigation_interface__msg__Stop__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_navigation_interface
navigation_interface__msg__Stop *
navigation_interface__msg__Stop__create(void);

/// Destroy msg/Stop message.
/**
 * It calls
 * navigation_interface__msg__Stop__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_navigation_interface
void
navigation_interface__msg__Stop__destroy(navigation_interface__msg__Stop * msg);

/// Check for msg/Stop message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_navigation_interface
bool
navigation_interface__msg__Stop__are_equal(const navigation_interface__msg__Stop * lhs, const navigation_interface__msg__Stop * rhs);

/// Copy a msg/Stop message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_navigation_interface
bool
navigation_interface__msg__Stop__copy(
  const navigation_interface__msg__Stop * input,
  navigation_interface__msg__Stop * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_navigation_interface
const rosidl_type_hash_t *
navigation_interface__msg__Stop__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_navigation_interface
const rosidl_runtime_c__type_description__TypeDescription *
navigation_interface__msg__Stop__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_navigation_interface
const rosidl_runtime_c__type_description__TypeSource *
navigation_interface__msg__Stop__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_navigation_interface
const rosidl_runtime_c__type_description__TypeSource__Sequence *
navigation_interface__msg__Stop__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/Stop messages.
/**
 * It allocates the memory for the number of elements and calls
 * navigation_interface__msg__Stop__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_navigation_interface
bool
navigation_interface__msg__Stop__Sequence__init(navigation_interface__msg__Stop__Sequence * array, size_t size);

/// Finalize array of msg/Stop messages.
/**
 * It calls
 * navigation_interface__msg__Stop__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_navigation_interface
void
navigation_interface__msg__Stop__Sequence__fini(navigation_interface__msg__Stop__Sequence * array);

/// Create array of msg/Stop messages.
/**
 * It allocates the memory for the array and calls
 * navigation_interface__msg__Stop__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_navigation_interface
navigation_interface__msg__Stop__Sequence *
navigation_interface__msg__Stop__Sequence__create(size_t size);

/// Destroy array of msg/Stop messages.
/**
 * It calls
 * navigation_interface__msg__Stop__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_navigation_interface
void
navigation_interface__msg__Stop__Sequence__destroy(navigation_interface__msg__Stop__Sequence * array);

/// Check for msg/Stop message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_navigation_interface
bool
navigation_interface__msg__Stop__Sequence__are_equal(const navigation_interface__msg__Stop__Sequence * lhs, const navigation_interface__msg__Stop__Sequence * rhs);

/// Copy an array of msg/Stop messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_navigation_interface
bool
navigation_interface__msg__Stop__Sequence__copy(
  const navigation_interface__msg__Stop__Sequence * input,
  navigation_interface__msg__Stop__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // NAVIGATION_INTERFACE__MSG__DETAIL__STOP__FUNCTIONS_H_
