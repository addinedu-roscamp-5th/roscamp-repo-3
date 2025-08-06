// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robocallee_fms:srv/CustomerRequest.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robocallee_fms/srv/customer_request.hpp"


#ifndef ROBOCALLEE_FMS__SRV__DETAIL__CUSTOMER_REQUEST__BUILDER_HPP_
#define ROBOCALLEE_FMS__SRV__DETAIL__CUSTOMER_REQUEST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robocallee_fms/srv/detail/customer_request__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robocallee_fms
{

namespace srv
{

namespace builder
{

class Init_CustomerRequest_Request_customer_id
{
public:
  explicit Init_CustomerRequest_Request_customer_id(::robocallee_fms::srv::CustomerRequest_Request & msg)
  : msg_(msg)
  {}
  ::robocallee_fms::srv::CustomerRequest_Request customer_id(::robocallee_fms::srv::CustomerRequest_Request::_customer_id_type arg)
  {
    msg_.customer_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robocallee_fms::srv::CustomerRequest_Request msg_;
};

class Init_CustomerRequest_Request_y
{
public:
  explicit Init_CustomerRequest_Request_y(::robocallee_fms::srv::CustomerRequest_Request & msg)
  : msg_(msg)
  {}
  Init_CustomerRequest_Request_customer_id y(::robocallee_fms::srv::CustomerRequest_Request::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_CustomerRequest_Request_customer_id(msg_);
  }

private:
  ::robocallee_fms::srv::CustomerRequest_Request msg_;
};

class Init_CustomerRequest_Request_x
{
public:
  explicit Init_CustomerRequest_Request_x(::robocallee_fms::srv::CustomerRequest_Request & msg)
  : msg_(msg)
  {}
  Init_CustomerRequest_Request_y x(::robocallee_fms::srv::CustomerRequest_Request::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_CustomerRequest_Request_y(msg_);
  }

private:
  ::robocallee_fms::srv::CustomerRequest_Request msg_;
};

class Init_CustomerRequest_Request_color
{
public:
  explicit Init_CustomerRequest_Request_color(::robocallee_fms::srv::CustomerRequest_Request & msg)
  : msg_(msg)
  {}
  Init_CustomerRequest_Request_x color(::robocallee_fms::srv::CustomerRequest_Request::_color_type arg)
  {
    msg_.color = std::move(arg);
    return Init_CustomerRequest_Request_x(msg_);
  }

private:
  ::robocallee_fms::srv::CustomerRequest_Request msg_;
};

class Init_CustomerRequest_Request_size
{
public:
  explicit Init_CustomerRequest_Request_size(::robocallee_fms::srv::CustomerRequest_Request & msg)
  : msg_(msg)
  {}
  Init_CustomerRequest_Request_color size(::robocallee_fms::srv::CustomerRequest_Request::_size_type arg)
  {
    msg_.size = std::move(arg);
    return Init_CustomerRequest_Request_color(msg_);
  }

private:
  ::robocallee_fms::srv::CustomerRequest_Request msg_;
};

class Init_CustomerRequest_Request_model
{
public:
  explicit Init_CustomerRequest_Request_model(::robocallee_fms::srv::CustomerRequest_Request & msg)
  : msg_(msg)
  {}
  Init_CustomerRequest_Request_size model(::robocallee_fms::srv::CustomerRequest_Request::_model_type arg)
  {
    msg_.model = std::move(arg);
    return Init_CustomerRequest_Request_size(msg_);
  }

private:
  ::robocallee_fms::srv::CustomerRequest_Request msg_;
};

class Init_CustomerRequest_Request_action
{
public:
  explicit Init_CustomerRequest_Request_action(::robocallee_fms::srv::CustomerRequest_Request & msg)
  : msg_(msg)
  {}
  Init_CustomerRequest_Request_model action(::robocallee_fms::srv::CustomerRequest_Request::_action_type arg)
  {
    msg_.action = std::move(arg);
    return Init_CustomerRequest_Request_model(msg_);
  }

private:
  ::robocallee_fms::srv::CustomerRequest_Request msg_;
};

class Init_CustomerRequest_Request_requester
{
public:
  Init_CustomerRequest_Request_requester()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CustomerRequest_Request_action requester(::robocallee_fms::srv::CustomerRequest_Request::_requester_type arg)
  {
    msg_.requester = std::move(arg);
    return Init_CustomerRequest_Request_action(msg_);
  }

private:
  ::robocallee_fms::srv::CustomerRequest_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robocallee_fms::srv::CustomerRequest_Request>()
{
  return robocallee_fms::srv::builder::Init_CustomerRequest_Request_requester();
}

}  // namespace robocallee_fms


namespace robocallee_fms
{

namespace srv
{

namespace builder
{

class Init_CustomerRequest_Response_success
{
public:
  explicit Init_CustomerRequest_Response_success(::robocallee_fms::srv::CustomerRequest_Response & msg)
  : msg_(msg)
  {}
  ::robocallee_fms::srv::CustomerRequest_Response success(::robocallee_fms::srv::CustomerRequest_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robocallee_fms::srv::CustomerRequest_Response msg_;
};

class Init_CustomerRequest_Response_action
{
public:
  explicit Init_CustomerRequest_Response_action(::robocallee_fms::srv::CustomerRequest_Response & msg)
  : msg_(msg)
  {}
  Init_CustomerRequest_Response_success action(::robocallee_fms::srv::CustomerRequest_Response::_action_type arg)
  {
    msg_.action = std::move(arg);
    return Init_CustomerRequest_Response_success(msg_);
  }

private:
  ::robocallee_fms::srv::CustomerRequest_Response msg_;
};

class Init_CustomerRequest_Response_wait_list
{
public:
  Init_CustomerRequest_Response_wait_list()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CustomerRequest_Response_action wait_list(::robocallee_fms::srv::CustomerRequest_Response::_wait_list_type arg)
  {
    msg_.wait_list = std::move(arg);
    return Init_CustomerRequest_Response_action(msg_);
  }

private:
  ::robocallee_fms::srv::CustomerRequest_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robocallee_fms::srv::CustomerRequest_Response>()
{
  return robocallee_fms::srv::builder::Init_CustomerRequest_Response_wait_list();
}

}  // namespace robocallee_fms


namespace robocallee_fms
{

namespace srv
{

namespace builder
{

class Init_CustomerRequest_Event_response
{
public:
  explicit Init_CustomerRequest_Event_response(::robocallee_fms::srv::CustomerRequest_Event & msg)
  : msg_(msg)
  {}
  ::robocallee_fms::srv::CustomerRequest_Event response(::robocallee_fms::srv::CustomerRequest_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robocallee_fms::srv::CustomerRequest_Event msg_;
};

class Init_CustomerRequest_Event_request
{
public:
  explicit Init_CustomerRequest_Event_request(::robocallee_fms::srv::CustomerRequest_Event & msg)
  : msg_(msg)
  {}
  Init_CustomerRequest_Event_response request(::robocallee_fms::srv::CustomerRequest_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_CustomerRequest_Event_response(msg_);
  }

private:
  ::robocallee_fms::srv::CustomerRequest_Event msg_;
};

class Init_CustomerRequest_Event_info
{
public:
  Init_CustomerRequest_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CustomerRequest_Event_request info(::robocallee_fms::srv::CustomerRequest_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_CustomerRequest_Event_request(msg_);
  }

private:
  ::robocallee_fms::srv::CustomerRequest_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robocallee_fms::srv::CustomerRequest_Event>()
{
  return robocallee_fms::srv::builder::Init_CustomerRequest_Event_info();
}

}  // namespace robocallee_fms

#endif  // ROBOCALLEE_FMS__SRV__DETAIL__CUSTOMER_REQUEST__BUILDER_HPP_
