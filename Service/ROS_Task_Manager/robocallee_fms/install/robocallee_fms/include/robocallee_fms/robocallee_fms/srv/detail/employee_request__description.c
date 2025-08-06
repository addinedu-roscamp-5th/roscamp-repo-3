// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from robocallee_fms:srv/EmployeeRequest.idl
// generated code does not contain a copyright notice

#include "robocallee_fms/srv/detail/employee_request__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_robocallee_fms
const rosidl_type_hash_t *
robocallee_fms__srv__EmployeeRequest__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x33, 0x6d, 0x73, 0xef, 0xbe, 0x0c, 0x8a, 0xda,
      0xec, 0x70, 0x75, 0x3d, 0xfb, 0x80, 0xb6, 0x73,
      0xe7, 0xef, 0x19, 0xe3, 0x14, 0xcb, 0xf7, 0xfa,
      0xbd, 0x01, 0xe5, 0x07, 0xf3, 0xf7, 0x4c, 0x99,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_robocallee_fms
const rosidl_type_hash_t *
robocallee_fms__srv__EmployeeRequest_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x47, 0xe9, 0x08, 0x04, 0x79, 0x56, 0x05, 0xfa,
      0x48, 0x99, 0x30, 0xf6, 0xa9, 0x1f, 0x15, 0x45,
      0xda, 0x7d, 0x63, 0x93, 0x1e, 0x0b, 0x38, 0xda,
      0xdd, 0xaf, 0xe3, 0x6e, 0xdc, 0xdf, 0x49, 0x0b,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_robocallee_fms
const rosidl_type_hash_t *
robocallee_fms__srv__EmployeeRequest_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x8a, 0x12, 0x48, 0x12, 0x6c, 0x37, 0x0e, 0x9d,
      0x3e, 0xfe, 0x56, 0xdb, 0x10, 0xa2, 0x53, 0x22,
      0x78, 0x5d, 0x8f, 0x83, 0x1a, 0x39, 0xa0, 0x13,
      0xa4, 0xac, 0x85, 0xf7, 0x6e, 0xa2, 0xbd, 0x19,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_robocallee_fms
const rosidl_type_hash_t *
robocallee_fms__srv__EmployeeRequest_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x14, 0xbf, 0x72, 0xcc, 0x05, 0xe2, 0xad, 0x32,
      0x92, 0xaf, 0xf1, 0x8e, 0xeb, 0x8c, 0x0e, 0x48,
      0xbb, 0x14, 0x62, 0x32, 0xd6, 0x8e, 0x17, 0x68,
      0x27, 0x69, 0x30, 0x8d, 0x2e, 0x34, 0x8c, 0x20,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "service_msgs/msg/detail/service_event_info__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char robocallee_fms__srv__EmployeeRequest__TYPE_NAME[] = "robocallee_fms/srv/EmployeeRequest";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char robocallee_fms__srv__EmployeeRequest_Event__TYPE_NAME[] = "robocallee_fms/srv/EmployeeRequest_Event";
static char robocallee_fms__srv__EmployeeRequest_Request__TYPE_NAME[] = "robocallee_fms/srv/EmployeeRequest_Request";
static char robocallee_fms__srv__EmployeeRequest_Response__TYPE_NAME[] = "robocallee_fms/srv/EmployeeRequest_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char robocallee_fms__srv__EmployeeRequest__FIELD_NAME__request_message[] = "request_message";
static char robocallee_fms__srv__EmployeeRequest__FIELD_NAME__response_message[] = "response_message";
static char robocallee_fms__srv__EmployeeRequest__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field robocallee_fms__srv__EmployeeRequest__FIELDS[] = {
  {
    {robocallee_fms__srv__EmployeeRequest__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {robocallee_fms__srv__EmployeeRequest_Request__TYPE_NAME, 42, 42},
    },
    {NULL, 0, 0},
  },
  {
    {robocallee_fms__srv__EmployeeRequest__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {robocallee_fms__srv__EmployeeRequest_Response__TYPE_NAME, 43, 43},
    },
    {NULL, 0, 0},
  },
  {
    {robocallee_fms__srv__EmployeeRequest__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {robocallee_fms__srv__EmployeeRequest_Event__TYPE_NAME, 40, 40},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription robocallee_fms__srv__EmployeeRequest__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {robocallee_fms__srv__EmployeeRequest_Event__TYPE_NAME, 40, 40},
    {NULL, 0, 0},
  },
  {
    {robocallee_fms__srv__EmployeeRequest_Request__TYPE_NAME, 42, 42},
    {NULL, 0, 0},
  },
  {
    {robocallee_fms__srv__EmployeeRequest_Response__TYPE_NAME, 43, 43},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
robocallee_fms__srv__EmployeeRequest__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {robocallee_fms__srv__EmployeeRequest__TYPE_NAME, 34, 34},
      {robocallee_fms__srv__EmployeeRequest__FIELDS, 3, 3},
    },
    {robocallee_fms__srv__EmployeeRequest__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = robocallee_fms__srv__EmployeeRequest_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = robocallee_fms__srv__EmployeeRequest_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = robocallee_fms__srv__EmployeeRequest_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char robocallee_fms__srv__EmployeeRequest_Request__FIELD_NAME__requester[] = "requester";
static char robocallee_fms__srv__EmployeeRequest_Request__FIELD_NAME__action[] = "action";

static rosidl_runtime_c__type_description__Field robocallee_fms__srv__EmployeeRequest_Request__FIELDS[] = {
  {
    {robocallee_fms__srv__EmployeeRequest_Request__FIELD_NAME__requester, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robocallee_fms__srv__EmployeeRequest_Request__FIELD_NAME__action, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
robocallee_fms__srv__EmployeeRequest_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {robocallee_fms__srv__EmployeeRequest_Request__TYPE_NAME, 42, 42},
      {robocallee_fms__srv__EmployeeRequest_Request__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char robocallee_fms__srv__EmployeeRequest_Response__FIELD_NAME__wait_list[] = "wait_list";
static char robocallee_fms__srv__EmployeeRequest_Response__FIELD_NAME__action[] = "action";
static char robocallee_fms__srv__EmployeeRequest_Response__FIELD_NAME__success[] = "success";

static rosidl_runtime_c__type_description__Field robocallee_fms__srv__EmployeeRequest_Response__FIELDS[] = {
  {
    {robocallee_fms__srv__EmployeeRequest_Response__FIELD_NAME__wait_list, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robocallee_fms__srv__EmployeeRequest_Response__FIELD_NAME__action, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robocallee_fms__srv__EmployeeRequest_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
robocallee_fms__srv__EmployeeRequest_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {robocallee_fms__srv__EmployeeRequest_Response__TYPE_NAME, 43, 43},
      {robocallee_fms__srv__EmployeeRequest_Response__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char robocallee_fms__srv__EmployeeRequest_Event__FIELD_NAME__info[] = "info";
static char robocallee_fms__srv__EmployeeRequest_Event__FIELD_NAME__request[] = "request";
static char robocallee_fms__srv__EmployeeRequest_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field robocallee_fms__srv__EmployeeRequest_Event__FIELDS[] = {
  {
    {robocallee_fms__srv__EmployeeRequest_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {robocallee_fms__srv__EmployeeRequest_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {robocallee_fms__srv__EmployeeRequest_Request__TYPE_NAME, 42, 42},
    },
    {NULL, 0, 0},
  },
  {
    {robocallee_fms__srv__EmployeeRequest_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {robocallee_fms__srv__EmployeeRequest_Response__TYPE_NAME, 43, 43},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription robocallee_fms__srv__EmployeeRequest_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {robocallee_fms__srv__EmployeeRequest_Request__TYPE_NAME, 42, 42},
    {NULL, 0, 0},
  },
  {
    {robocallee_fms__srv__EmployeeRequest_Response__TYPE_NAME, 43, 43},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
robocallee_fms__srv__EmployeeRequest_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {robocallee_fms__srv__EmployeeRequest_Event__TYPE_NAME, 40, 40},
      {robocallee_fms__srv__EmployeeRequest_Event__FIELDS, 3, 3},
    },
    {robocallee_fms__srv__EmployeeRequest_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = robocallee_fms__srv__EmployeeRequest_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = robocallee_fms__srv__EmployeeRequest_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string requester\n"
  "string action\n"
  "---\n"
  "int32 wait_list\n"
  "string action\n"
  "bool success";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
robocallee_fms__srv__EmployeeRequest__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {robocallee_fms__srv__EmployeeRequest__TYPE_NAME, 34, 34},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 77, 77},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
robocallee_fms__srv__EmployeeRequest_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {robocallee_fms__srv__EmployeeRequest_Request__TYPE_NAME, 42, 42},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
robocallee_fms__srv__EmployeeRequest_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {robocallee_fms__srv__EmployeeRequest_Response__TYPE_NAME, 43, 43},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
robocallee_fms__srv__EmployeeRequest_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {robocallee_fms__srv__EmployeeRequest_Event__TYPE_NAME, 40, 40},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
robocallee_fms__srv__EmployeeRequest__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *robocallee_fms__srv__EmployeeRequest__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *robocallee_fms__srv__EmployeeRequest_Event__get_individual_type_description_source(NULL);
    sources[3] = *robocallee_fms__srv__EmployeeRequest_Request__get_individual_type_description_source(NULL);
    sources[4] = *robocallee_fms__srv__EmployeeRequest_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
robocallee_fms__srv__EmployeeRequest_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *robocallee_fms__srv__EmployeeRequest_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
robocallee_fms__srv__EmployeeRequest_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *robocallee_fms__srv__EmployeeRequest_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
robocallee_fms__srv__EmployeeRequest_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *robocallee_fms__srv__EmployeeRequest_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *robocallee_fms__srv__EmployeeRequest_Request__get_individual_type_description_source(NULL);
    sources[3] = *robocallee_fms__srv__EmployeeRequest_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
