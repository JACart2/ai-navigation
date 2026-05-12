// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from motor_control_interface:msg/VelAngle.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "motor_control_interface/msg/vel_angle.h"


#ifndef MOTOR_CONTROL_INTERFACE__MSG__DETAIL__VEL_ANGLE__FUNCTIONS_H_
#define MOTOR_CONTROL_INTERFACE__MSG__DETAIL__VEL_ANGLE__FUNCTIONS_H_

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
#include "motor_control_interface/msg/rosidl_generator_c__visibility_control.h"

#include "motor_control_interface/msg/detail/vel_angle__struct.h"

/// Initialize msg/VelAngle message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * motor_control_interface__msg__VelAngle
 * )) before or use
 * motor_control_interface__msg__VelAngle__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_control_interface
bool
motor_control_interface__msg__VelAngle__init(motor_control_interface__msg__VelAngle * msg);

/// Finalize msg/VelAngle message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_control_interface
void
motor_control_interface__msg__VelAngle__fini(motor_control_interface__msg__VelAngle * msg);

/// Create msg/VelAngle message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * motor_control_interface__msg__VelAngle__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_control_interface
motor_control_interface__msg__VelAngle *
motor_control_interface__msg__VelAngle__create(void);

/// Destroy msg/VelAngle message.
/**
 * It calls
 * motor_control_interface__msg__VelAngle__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_control_interface
void
motor_control_interface__msg__VelAngle__destroy(motor_control_interface__msg__VelAngle * msg);

/// Check for msg/VelAngle message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_control_interface
bool
motor_control_interface__msg__VelAngle__are_equal(const motor_control_interface__msg__VelAngle * lhs, const motor_control_interface__msg__VelAngle * rhs);

/// Copy a msg/VelAngle message.
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
ROSIDL_GENERATOR_C_PUBLIC_motor_control_interface
bool
motor_control_interface__msg__VelAngle__copy(
  const motor_control_interface__msg__VelAngle * input,
  motor_control_interface__msg__VelAngle * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_motor_control_interface
const rosidl_type_hash_t *
motor_control_interface__msg__VelAngle__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_motor_control_interface
const rosidl_runtime_c__type_description__TypeDescription *
motor_control_interface__msg__VelAngle__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_motor_control_interface
const rosidl_runtime_c__type_description__TypeSource *
motor_control_interface__msg__VelAngle__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_motor_control_interface
const rosidl_runtime_c__type_description__TypeSource__Sequence *
motor_control_interface__msg__VelAngle__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/VelAngle messages.
/**
 * It allocates the memory for the number of elements and calls
 * motor_control_interface__msg__VelAngle__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_control_interface
bool
motor_control_interface__msg__VelAngle__Sequence__init(motor_control_interface__msg__VelAngle__Sequence * array, size_t size);

/// Finalize array of msg/VelAngle messages.
/**
 * It calls
 * motor_control_interface__msg__VelAngle__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_control_interface
void
motor_control_interface__msg__VelAngle__Sequence__fini(motor_control_interface__msg__VelAngle__Sequence * array);

/// Create array of msg/VelAngle messages.
/**
 * It allocates the memory for the array and calls
 * motor_control_interface__msg__VelAngle__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_control_interface
motor_control_interface__msg__VelAngle__Sequence *
motor_control_interface__msg__VelAngle__Sequence__create(size_t size);

/// Destroy array of msg/VelAngle messages.
/**
 * It calls
 * motor_control_interface__msg__VelAngle__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_control_interface
void
motor_control_interface__msg__VelAngle__Sequence__destroy(motor_control_interface__msg__VelAngle__Sequence * array);

/// Check for msg/VelAngle message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_control_interface
bool
motor_control_interface__msg__VelAngle__Sequence__are_equal(const motor_control_interface__msg__VelAngle__Sequence * lhs, const motor_control_interface__msg__VelAngle__Sequence * rhs);

/// Copy an array of msg/VelAngle messages.
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
ROSIDL_GENERATOR_C_PUBLIC_motor_control_interface
bool
motor_control_interface__msg__VelAngle__Sequence__copy(
  const motor_control_interface__msg__VelAngle__Sequence * input,
  motor_control_interface__msg__VelAngle__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // MOTOR_CONTROL_INTERFACE__MSG__DETAIL__VEL_ANGLE__FUNCTIONS_H_
