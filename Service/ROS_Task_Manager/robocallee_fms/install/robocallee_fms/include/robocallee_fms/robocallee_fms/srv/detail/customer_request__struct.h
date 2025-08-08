// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robocallee_fms:srv/CustomerRequest.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robocallee_fms/srv/customer_request.h"


#ifndef ROBOCALLEE_FMS__SRV__DETAIL__CUSTOMER_REQUEST__STRUCT_H_
#define ROBOCALLEE_FMS__SRV__DETAIL__CUSTOMER_REQUEST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'requester'
// Member 'action'
// Member 'model'
// Member 'color'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/CustomerRequest in the package robocallee_fms.
typedef struct robocallee_fms__srv__CustomerRequest_Request
{
  rosidl_runtime_c__String requester;
  rosidl_runtime_c__String action;
  rosidl_runtime_c__String model;
  int32_t size;
  rosidl_runtime_c__String color;
  float x;
  float y;
  int32_t customer_id;
} robocallee_fms__srv__CustomerRequest_Request;

// Struct for a sequence of robocallee_fms__srv__CustomerRequest_Request.
typedef struct robocallee_fms__srv__CustomerRequest_Request__Sequence
{
  robocallee_fms__srv__CustomerRequest_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robocallee_fms__srv__CustomerRequest_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'action'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/CustomerRequest in the package robocallee_fms.
typedef struct robocallee_fms__srv__CustomerRequest_Response
{
  int32_t wait_list;
  rosidl_runtime_c__String action;
  bool success;
} robocallee_fms__srv__CustomerRequest_Response;

// Struct for a sequence of robocallee_fms__srv__CustomerRequest_Response.
typedef struct robocallee_fms__srv__CustomerRequest_Response__Sequence
{
  robocallee_fms__srv__CustomerRequest_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robocallee_fms__srv__CustomerRequest_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  robocallee_fms__srv__CustomerRequest_Event__request__MAX_SIZE = 1
};
// response
enum
{
  robocallee_fms__srv__CustomerRequest_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/CustomerRequest in the package robocallee_fms.
typedef struct robocallee_fms__srv__CustomerRequest_Event
{
  service_msgs__msg__ServiceEventInfo info;
  robocallee_fms__srv__CustomerRequest_Request__Sequence request;
  robocallee_fms__srv__CustomerRequest_Response__Sequence response;
} robocallee_fms__srv__CustomerRequest_Event;

// Struct for a sequence of robocallee_fms__srv__CustomerRequest_Event.
typedef struct robocallee_fms__srv__CustomerRequest_Event__Sequence
{
  robocallee_fms__srv__CustomerRequest_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robocallee_fms__srv__CustomerRequest_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOCALLEE_FMS__SRV__DETAIL__CUSTOMER_REQUEST__STRUCT_H_
