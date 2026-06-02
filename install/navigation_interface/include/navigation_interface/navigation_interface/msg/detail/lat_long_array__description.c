// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from navigation_interface:msg/LatLongArray.idl
// generated code does not contain a copyright notice

#include "navigation_interface/msg/detail/lat_long_array__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_navigation_interface
const rosidl_type_hash_t *
navigation_interface__msg__LatLongArray__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x93, 0xf7, 0xe8, 0xbd, 0xcd, 0x8e, 0x89, 0xc2,
      0x4f, 0x58, 0x52, 0x31, 0xc5, 0xe6, 0xe0, 0x6e,
      0x17, 0x44, 0xe2, 0xab, 0xb4, 0x35, 0xb8, 0xa1,
      0x27, 0x2b, 0x05, 0xbc, 0xed, 0x9b, 0x08, 0x1c,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "std_msgs/msg/detail/header__functions.h"
#include "navigation_interface/msg/detail/lat_long_point__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t navigation_interface__msg__LatLongPoint__EXPECTED_HASH = {1, {
    0x19, 0xff, 0x2c, 0x34, 0x4e, 0x9e, 0x81, 0x81,
    0x8f, 0xfb, 0x36, 0x76, 0x82, 0x65, 0x6e, 0x32,
    0x02, 0x1d, 0xdd, 0x62, 0x39, 0x18, 0xab, 0x32,
    0x11, 0xce, 0x87, 0x79, 0x64, 0x94, 0xaa, 0x0d,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char navigation_interface__msg__LatLongArray__TYPE_NAME[] = "navigation_interface/msg/LatLongArray";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char navigation_interface__msg__LatLongPoint__TYPE_NAME[] = "navigation_interface/msg/LatLongPoint";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char navigation_interface__msg__LatLongArray__FIELD_NAME__header[] = "header";
static char navigation_interface__msg__LatLongArray__FIELD_NAME__gpspoints[] = "gpspoints";

static rosidl_runtime_c__type_description__Field navigation_interface__msg__LatLongArray__FIELDS[] = {
  {
    {navigation_interface__msg__LatLongArray__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {navigation_interface__msg__LatLongArray__FIELD_NAME__gpspoints, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {navigation_interface__msg__LatLongPoint__TYPE_NAME, 37, 37},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription navigation_interface__msg__LatLongArray__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {navigation_interface__msg__LatLongPoint__TYPE_NAME, 37, 37},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
navigation_interface__msg__LatLongArray__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {navigation_interface__msg__LatLongArray__TYPE_NAME, 37, 37},
      {navigation_interface__msg__LatLongArray__FIELDS, 2, 2},
    },
    {navigation_interface__msg__LatLongArray__REFERENCED_TYPE_DESCRIPTIONS, 3, 3},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&navigation_interface__msg__LatLongPoint__EXPECTED_HASH, navigation_interface__msg__LatLongPoint__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = navigation_interface__msg__LatLongPoint__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "std_msgs/Header header\n"
  "LatLongPoint[] gpspoints";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
navigation_interface__msg__LatLongArray__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {navigation_interface__msg__LatLongArray__TYPE_NAME, 37, 37},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 47, 47},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
navigation_interface__msg__LatLongArray__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[4];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 4, 4};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *navigation_interface__msg__LatLongArray__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *navigation_interface__msg__LatLongPoint__get_individual_type_description_source(NULL);
    sources[3] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
