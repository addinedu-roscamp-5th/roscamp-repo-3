// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robocallee_fms:srv/CustomerRequest.idl
// generated code does not contain a copyright notice
#include "robocallee_fms/srv/detail/customer_request__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `requester`
// Member `action`
// Member `model`
// Member `color`
#include "rosidl_runtime_c/string_functions.h"

bool
robocallee_fms__srv__CustomerRequest_Request__init(robocallee_fms__srv__CustomerRequest_Request * msg)
{
  if (!msg) {
    return false;
  }
  // requester
  if (!rosidl_runtime_c__String__init(&msg->requester)) {
    robocallee_fms__srv__CustomerRequest_Request__fini(msg);
    return false;
  }
  // action
  if (!rosidl_runtime_c__String__init(&msg->action)) {
    robocallee_fms__srv__CustomerRequest_Request__fini(msg);
    return false;
  }
  // model
  if (!rosidl_runtime_c__String__init(&msg->model)) {
    robocallee_fms__srv__CustomerRequest_Request__fini(msg);
    return false;
  }
  // size
  // color
  if (!rosidl_runtime_c__String__init(&msg->color)) {
    robocallee_fms__srv__CustomerRequest_Request__fini(msg);
    return false;
  }
  // x
  // y
  // customer_id
  return true;
}

void
robocallee_fms__srv__CustomerRequest_Request__fini(robocallee_fms__srv__CustomerRequest_Request * msg)
{
  if (!msg) {
    return;
  }
  // requester
  rosidl_runtime_c__String__fini(&msg->requester);
  // action
  rosidl_runtime_c__String__fini(&msg->action);
  // model
  rosidl_runtime_c__String__fini(&msg->model);
  // size
  // color
  rosidl_runtime_c__String__fini(&msg->color);
  // x
  // y
  // customer_id
}

bool
robocallee_fms__srv__CustomerRequest_Request__are_equal(const robocallee_fms__srv__CustomerRequest_Request * lhs, const robocallee_fms__srv__CustomerRequest_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // requester
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->requester), &(rhs->requester)))
  {
    return false;
  }
  // action
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->action), &(rhs->action)))
  {
    return false;
  }
  // model
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->model), &(rhs->model)))
  {
    return false;
  }
  // size
  if (lhs->size != rhs->size) {
    return false;
  }
  // color
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->color), &(rhs->color)))
  {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // customer_id
  if (lhs->customer_id != rhs->customer_id) {
    return false;
  }
  return true;
}

bool
robocallee_fms__srv__CustomerRequest_Request__copy(
  const robocallee_fms__srv__CustomerRequest_Request * input,
  robocallee_fms__srv__CustomerRequest_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // requester
  if (!rosidl_runtime_c__String__copy(
      &(input->requester), &(output->requester)))
  {
    return false;
  }
  // action
  if (!rosidl_runtime_c__String__copy(
      &(input->action), &(output->action)))
  {
    return false;
  }
  // model
  if (!rosidl_runtime_c__String__copy(
      &(input->model), &(output->model)))
  {
    return false;
  }
  // size
  output->size = input->size;
  // color
  if (!rosidl_runtime_c__String__copy(
      &(input->color), &(output->color)))
  {
    return false;
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // customer_id
  output->customer_id = input->customer_id;
  return true;
}

robocallee_fms__srv__CustomerRequest_Request *
robocallee_fms__srv__CustomerRequest_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robocallee_fms__srv__CustomerRequest_Request * msg = (robocallee_fms__srv__CustomerRequest_Request *)allocator.allocate(sizeof(robocallee_fms__srv__CustomerRequest_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robocallee_fms__srv__CustomerRequest_Request));
  bool success = robocallee_fms__srv__CustomerRequest_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robocallee_fms__srv__CustomerRequest_Request__destroy(robocallee_fms__srv__CustomerRequest_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robocallee_fms__srv__CustomerRequest_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robocallee_fms__srv__CustomerRequest_Request__Sequence__init(robocallee_fms__srv__CustomerRequest_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robocallee_fms__srv__CustomerRequest_Request * data = NULL;

  if (size) {
    data = (robocallee_fms__srv__CustomerRequest_Request *)allocator.zero_allocate(size, sizeof(robocallee_fms__srv__CustomerRequest_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robocallee_fms__srv__CustomerRequest_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robocallee_fms__srv__CustomerRequest_Request__fini(&data[i - 1]);
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
robocallee_fms__srv__CustomerRequest_Request__Sequence__fini(robocallee_fms__srv__CustomerRequest_Request__Sequence * array)
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
      robocallee_fms__srv__CustomerRequest_Request__fini(&array->data[i]);
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

robocallee_fms__srv__CustomerRequest_Request__Sequence *
robocallee_fms__srv__CustomerRequest_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robocallee_fms__srv__CustomerRequest_Request__Sequence * array = (robocallee_fms__srv__CustomerRequest_Request__Sequence *)allocator.allocate(sizeof(robocallee_fms__srv__CustomerRequest_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robocallee_fms__srv__CustomerRequest_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robocallee_fms__srv__CustomerRequest_Request__Sequence__destroy(robocallee_fms__srv__CustomerRequest_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robocallee_fms__srv__CustomerRequest_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robocallee_fms__srv__CustomerRequest_Request__Sequence__are_equal(const robocallee_fms__srv__CustomerRequest_Request__Sequence * lhs, const robocallee_fms__srv__CustomerRequest_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robocallee_fms__srv__CustomerRequest_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robocallee_fms__srv__CustomerRequest_Request__Sequence__copy(
  const robocallee_fms__srv__CustomerRequest_Request__Sequence * input,
  robocallee_fms__srv__CustomerRequest_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robocallee_fms__srv__CustomerRequest_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robocallee_fms__srv__CustomerRequest_Request * data =
      (robocallee_fms__srv__CustomerRequest_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robocallee_fms__srv__CustomerRequest_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robocallee_fms__srv__CustomerRequest_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robocallee_fms__srv__CustomerRequest_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `action`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
robocallee_fms__srv__CustomerRequest_Response__init(robocallee_fms__srv__CustomerRequest_Response * msg)
{
  if (!msg) {
    return false;
  }
  // wait_list
  // action
  if (!rosidl_runtime_c__String__init(&msg->action)) {
    robocallee_fms__srv__CustomerRequest_Response__fini(msg);
    return false;
  }
  // success
  return true;
}

void
robocallee_fms__srv__CustomerRequest_Response__fini(robocallee_fms__srv__CustomerRequest_Response * msg)
{
  if (!msg) {
    return;
  }
  // wait_list
  // action
  rosidl_runtime_c__String__fini(&msg->action);
  // success
}

bool
robocallee_fms__srv__CustomerRequest_Response__are_equal(const robocallee_fms__srv__CustomerRequest_Response * lhs, const robocallee_fms__srv__CustomerRequest_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // wait_list
  if (lhs->wait_list != rhs->wait_list) {
    return false;
  }
  // action
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->action), &(rhs->action)))
  {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
robocallee_fms__srv__CustomerRequest_Response__copy(
  const robocallee_fms__srv__CustomerRequest_Response * input,
  robocallee_fms__srv__CustomerRequest_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // wait_list
  output->wait_list = input->wait_list;
  // action
  if (!rosidl_runtime_c__String__copy(
      &(input->action), &(output->action)))
  {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

robocallee_fms__srv__CustomerRequest_Response *
robocallee_fms__srv__CustomerRequest_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robocallee_fms__srv__CustomerRequest_Response * msg = (robocallee_fms__srv__CustomerRequest_Response *)allocator.allocate(sizeof(robocallee_fms__srv__CustomerRequest_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robocallee_fms__srv__CustomerRequest_Response));
  bool success = robocallee_fms__srv__CustomerRequest_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robocallee_fms__srv__CustomerRequest_Response__destroy(robocallee_fms__srv__CustomerRequest_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robocallee_fms__srv__CustomerRequest_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robocallee_fms__srv__CustomerRequest_Response__Sequence__init(robocallee_fms__srv__CustomerRequest_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robocallee_fms__srv__CustomerRequest_Response * data = NULL;

  if (size) {
    data = (robocallee_fms__srv__CustomerRequest_Response *)allocator.zero_allocate(size, sizeof(robocallee_fms__srv__CustomerRequest_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robocallee_fms__srv__CustomerRequest_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robocallee_fms__srv__CustomerRequest_Response__fini(&data[i - 1]);
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
robocallee_fms__srv__CustomerRequest_Response__Sequence__fini(robocallee_fms__srv__CustomerRequest_Response__Sequence * array)
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
      robocallee_fms__srv__CustomerRequest_Response__fini(&array->data[i]);
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

robocallee_fms__srv__CustomerRequest_Response__Sequence *
robocallee_fms__srv__CustomerRequest_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robocallee_fms__srv__CustomerRequest_Response__Sequence * array = (robocallee_fms__srv__CustomerRequest_Response__Sequence *)allocator.allocate(sizeof(robocallee_fms__srv__CustomerRequest_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robocallee_fms__srv__CustomerRequest_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robocallee_fms__srv__CustomerRequest_Response__Sequence__destroy(robocallee_fms__srv__CustomerRequest_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robocallee_fms__srv__CustomerRequest_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robocallee_fms__srv__CustomerRequest_Response__Sequence__are_equal(const robocallee_fms__srv__CustomerRequest_Response__Sequence * lhs, const robocallee_fms__srv__CustomerRequest_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robocallee_fms__srv__CustomerRequest_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robocallee_fms__srv__CustomerRequest_Response__Sequence__copy(
  const robocallee_fms__srv__CustomerRequest_Response__Sequence * input,
  robocallee_fms__srv__CustomerRequest_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robocallee_fms__srv__CustomerRequest_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robocallee_fms__srv__CustomerRequest_Response * data =
      (robocallee_fms__srv__CustomerRequest_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robocallee_fms__srv__CustomerRequest_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robocallee_fms__srv__CustomerRequest_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robocallee_fms__srv__CustomerRequest_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "robocallee_fms/srv/detail/customer_request__functions.h"

bool
robocallee_fms__srv__CustomerRequest_Event__init(robocallee_fms__srv__CustomerRequest_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    robocallee_fms__srv__CustomerRequest_Event__fini(msg);
    return false;
  }
  // request
  if (!robocallee_fms__srv__CustomerRequest_Request__Sequence__init(&msg->request, 0)) {
    robocallee_fms__srv__CustomerRequest_Event__fini(msg);
    return false;
  }
  // response
  if (!robocallee_fms__srv__CustomerRequest_Response__Sequence__init(&msg->response, 0)) {
    robocallee_fms__srv__CustomerRequest_Event__fini(msg);
    return false;
  }
  return true;
}

void
robocallee_fms__srv__CustomerRequest_Event__fini(robocallee_fms__srv__CustomerRequest_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  robocallee_fms__srv__CustomerRequest_Request__Sequence__fini(&msg->request);
  // response
  robocallee_fms__srv__CustomerRequest_Response__Sequence__fini(&msg->response);
}

bool
robocallee_fms__srv__CustomerRequest_Event__are_equal(const robocallee_fms__srv__CustomerRequest_Event * lhs, const robocallee_fms__srv__CustomerRequest_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!robocallee_fms__srv__CustomerRequest_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!robocallee_fms__srv__CustomerRequest_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
robocallee_fms__srv__CustomerRequest_Event__copy(
  const robocallee_fms__srv__CustomerRequest_Event * input,
  robocallee_fms__srv__CustomerRequest_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!robocallee_fms__srv__CustomerRequest_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!robocallee_fms__srv__CustomerRequest_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

robocallee_fms__srv__CustomerRequest_Event *
robocallee_fms__srv__CustomerRequest_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robocallee_fms__srv__CustomerRequest_Event * msg = (robocallee_fms__srv__CustomerRequest_Event *)allocator.allocate(sizeof(robocallee_fms__srv__CustomerRequest_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robocallee_fms__srv__CustomerRequest_Event));
  bool success = robocallee_fms__srv__CustomerRequest_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robocallee_fms__srv__CustomerRequest_Event__destroy(robocallee_fms__srv__CustomerRequest_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robocallee_fms__srv__CustomerRequest_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robocallee_fms__srv__CustomerRequest_Event__Sequence__init(robocallee_fms__srv__CustomerRequest_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robocallee_fms__srv__CustomerRequest_Event * data = NULL;

  if (size) {
    data = (robocallee_fms__srv__CustomerRequest_Event *)allocator.zero_allocate(size, sizeof(robocallee_fms__srv__CustomerRequest_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robocallee_fms__srv__CustomerRequest_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robocallee_fms__srv__CustomerRequest_Event__fini(&data[i - 1]);
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
robocallee_fms__srv__CustomerRequest_Event__Sequence__fini(robocallee_fms__srv__CustomerRequest_Event__Sequence * array)
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
      robocallee_fms__srv__CustomerRequest_Event__fini(&array->data[i]);
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

robocallee_fms__srv__CustomerRequest_Event__Sequence *
robocallee_fms__srv__CustomerRequest_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robocallee_fms__srv__CustomerRequest_Event__Sequence * array = (robocallee_fms__srv__CustomerRequest_Event__Sequence *)allocator.allocate(sizeof(robocallee_fms__srv__CustomerRequest_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robocallee_fms__srv__CustomerRequest_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robocallee_fms__srv__CustomerRequest_Event__Sequence__destroy(robocallee_fms__srv__CustomerRequest_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robocallee_fms__srv__CustomerRequest_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robocallee_fms__srv__CustomerRequest_Event__Sequence__are_equal(const robocallee_fms__srv__CustomerRequest_Event__Sequence * lhs, const robocallee_fms__srv__CustomerRequest_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robocallee_fms__srv__CustomerRequest_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robocallee_fms__srv__CustomerRequest_Event__Sequence__copy(
  const robocallee_fms__srv__CustomerRequest_Event__Sequence * input,
  robocallee_fms__srv__CustomerRequest_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robocallee_fms__srv__CustomerRequest_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robocallee_fms__srv__CustomerRequest_Event * data =
      (robocallee_fms__srv__CustomerRequest_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robocallee_fms__srv__CustomerRequest_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robocallee_fms__srv__CustomerRequest_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robocallee_fms__srv__CustomerRequest_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
