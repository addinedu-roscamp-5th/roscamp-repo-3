// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robocallee_fms:srv/EmployeeRequest.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robocallee_fms/srv/employee_request.hpp"


#ifndef ROBOCALLEE_FMS__SRV__DETAIL__EMPLOYEE_REQUEST__BUILDER_HPP_
#define ROBOCALLEE_FMS__SRV__DETAIL__EMPLOYEE_REQUEST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robocallee_fms/srv/detail/employee_request__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robocallee_fms
{

namespace srv
{

namespace builder
{

class Init_EmployeeRequest_Request_action
{
public:
  explicit Init_EmployeeRequest_Request_action(::robocallee_fms::srv::EmployeeRequest_Request & msg)
  : msg_(msg)
  {}
  ::robocallee_fms::srv::EmployeeRequest_Request action(::robocallee_fms::srv::EmployeeRequest_Request::_action_type arg)
  {
    msg_.action = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robocallee_fms::srv::EmployeeRequest_Request msg_;
};

class Init_EmployeeRequest_Request_requester
{
public:
  Init_EmployeeRequest_Request_requester()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EmployeeRequest_Request_action requester(::robocallee_fms::srv::EmployeeRequest_Request::_requester_type arg)
  {
    msg_.requester = std::move(arg);
    return Init_EmployeeRequest_Request_action(msg_);
  }

private:
  ::robocallee_fms::srv::EmployeeRequest_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robocallee_fms::srv::EmployeeRequest_Request>()
{
  return robocallee_fms::srv::builder::Init_EmployeeRequest_Request_requester();
}

}  // namespace robocallee_fms


namespace robocallee_fms
{

namespace srv
{

namespace builder
{

class Init_EmployeeRequest_Response_success
{
public:
  explicit Init_EmployeeRequest_Response_success(::robocallee_fms::srv::EmployeeRequest_Response & msg)
  : msg_(msg)
  {}
  ::robocallee_fms::srv::EmployeeRequest_Response success(::robocallee_fms::srv::EmployeeRequest_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robocallee_fms::srv::EmployeeRequest_Response msg_;
};

class Init_EmployeeRequest_Response_action
{
public:
  explicit Init_EmployeeRequest_Response_action(::robocallee_fms::srv::EmployeeRequest_Response & msg)
  : msg_(msg)
  {}
  Init_EmployeeRequest_Response_success action(::robocallee_fms::srv::EmployeeRequest_Response::_action_type arg)
  {
    msg_.action = std::move(arg);
    return Init_EmployeeRequest_Response_success(msg_);
  }

private:
  ::robocallee_fms::srv::EmployeeRequest_Response msg_;
};

class Init_EmployeeRequest_Response_wait_list
{
public:
  Init_EmployeeRequest_Response_wait_list()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EmployeeRequest_Response_action wait_list(::robocallee_fms::srv::EmployeeRequest_Response::_wait_list_type arg)
  {
    msg_.wait_list = std::move(arg);
    return Init_EmployeeRequest_Response_action(msg_);
  }

private:
  ::robocallee_fms::srv::EmployeeRequest_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robocallee_fms::srv::EmployeeRequest_Response>()
{
  return robocallee_fms::srv::builder::Init_EmployeeRequest_Response_wait_list();
}

}  // namespace robocallee_fms


namespace robocallee_fms
{

namespace srv
{

namespace builder
{

class Init_EmployeeRequest_Event_response
{
public:
  explicit Init_EmployeeRequest_Event_response(::robocallee_fms::srv::EmployeeRequest_Event & msg)
  : msg_(msg)
  {}
  ::robocallee_fms::srv::EmployeeRequest_Event response(::robocallee_fms::srv::EmployeeRequest_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robocallee_fms::srv::EmployeeRequest_Event msg_;
};

class Init_EmployeeRequest_Event_request
{
public:
  explicit Init_EmployeeRequest_Event_request(::robocallee_fms::srv::EmployeeRequest_Event & msg)
  : msg_(msg)
  {}
  Init_EmployeeRequest_Event_response request(::robocallee_fms::srv::EmployeeRequest_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_EmployeeRequest_Event_response(msg_);
  }

private:
  ::robocallee_fms::srv::EmployeeRequest_Event msg_;
};

class Init_EmployeeRequest_Event_info
{
public:
  Init_EmployeeRequest_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EmployeeRequest_Event_request info(::robocallee_fms::srv::EmployeeRequest_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_EmployeeRequest_Event_request(msg_);
  }

private:
  ::robocallee_fms::srv::EmployeeRequest_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robocallee_fms::srv::EmployeeRequest_Event>()
{
  return robocallee_fms::srv::builder::Init_EmployeeRequest_Event_info();
}

}  // namespace robocallee_fms

#endif  // ROBOCALLEE_FMS__SRV__DETAIL__EMPLOYEE_REQUEST__BUILDER_HPP_
