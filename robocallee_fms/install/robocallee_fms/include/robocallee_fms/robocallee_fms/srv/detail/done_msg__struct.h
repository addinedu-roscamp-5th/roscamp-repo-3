// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robocallee_fms:srv/DoneMsg.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robocallee_fms/srv/done_msg.h"


#ifndef ROBOCALLEE_FMS__SRV__DETAIL__DONE_MSG__STRUCT_H_
#define ROBOCALLEE_FMS__SRV__DETAIL__DONE_MSG__STRUCT_H_

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
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/DoneMsg in the package robocallee_fms.
typedef struct robocallee_fms__srv__DoneMsg_Request
{
  rosidl_runtime_c__String requester;
  int32_t customer_id;
} robocallee_fms__srv__DoneMsg_Request;

// Struct for a sequence of robocallee_fms__srv__DoneMsg_Request.
typedef struct robocallee_fms__srv__DoneMsg_Request__Sequence
{
  robocallee_fms__srv__DoneMsg_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robocallee_fms__srv__DoneMsg_Request__Sequence;

// Constants defined in the message

/// Struct defined in srv/DoneMsg in the package robocallee_fms.
typedef struct robocallee_fms__srv__DoneMsg_Response
{
  bool accepted;
} robocallee_fms__srv__DoneMsg_Response;

// Struct for a sequence of robocallee_fms__srv__DoneMsg_Response.
typedef struct robocallee_fms__srv__DoneMsg_Response__Sequence
{
  robocallee_fms__srv__DoneMsg_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robocallee_fms__srv__DoneMsg_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  robocallee_fms__srv__DoneMsg_Event__request__MAX_SIZE = 1
};
// response
enum
{
  robocallee_fms__srv__DoneMsg_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/DoneMsg in the package robocallee_fms.
typedef struct robocallee_fms__srv__DoneMsg_Event
{
  service_msgs__msg__ServiceEventInfo info;
  robocallee_fms__srv__DoneMsg_Request__Sequence request;
  robocallee_fms__srv__DoneMsg_Response__Sequence response;
} robocallee_fms__srv__DoneMsg_Event;

// Struct for a sequence of robocallee_fms__srv__DoneMsg_Event.
typedef struct robocallee_fms__srv__DoneMsg_Event__Sequence
{
  robocallee_fms__srv__DoneMsg_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robocallee_fms__srv__DoneMsg_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOCALLEE_FMS__SRV__DETAIL__DONE_MSG__STRUCT_H_
