// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robocallee_fms:srv/DoneMsg.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robocallee_fms/srv/done_msg.hpp"


#ifndef ROBOCALLEE_FMS__SRV__DETAIL__DONE_MSG__BUILDER_HPP_
#define ROBOCALLEE_FMS__SRV__DETAIL__DONE_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robocallee_fms/srv/detail/done_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robocallee_fms
{

namespace srv
{

namespace builder
{

class Init_DoneMsg_Request_customer_id
{
public:
  explicit Init_DoneMsg_Request_customer_id(::robocallee_fms::srv::DoneMsg_Request & msg)
  : msg_(msg)
  {}
  ::robocallee_fms::srv::DoneMsg_Request customer_id(::robocallee_fms::srv::DoneMsg_Request::_customer_id_type arg)
  {
    msg_.customer_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robocallee_fms::srv::DoneMsg_Request msg_;
};

class Init_DoneMsg_Request_requester
{
public:
  Init_DoneMsg_Request_requester()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DoneMsg_Request_customer_id requester(::robocallee_fms::srv::DoneMsg_Request::_requester_type arg)
  {
    msg_.requester = std::move(arg);
    return Init_DoneMsg_Request_customer_id(msg_);
  }

private:
  ::robocallee_fms::srv::DoneMsg_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robocallee_fms::srv::DoneMsg_Request>()
{
  return robocallee_fms::srv::builder::Init_DoneMsg_Request_requester();
}

}  // namespace robocallee_fms


namespace robocallee_fms
{

namespace srv
{

namespace builder
{

class Init_DoneMsg_Response_accepted
{
public:
  Init_DoneMsg_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robocallee_fms::srv::DoneMsg_Response accepted(::robocallee_fms::srv::DoneMsg_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robocallee_fms::srv::DoneMsg_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robocallee_fms::srv::DoneMsg_Response>()
{
  return robocallee_fms::srv::builder::Init_DoneMsg_Response_accepted();
}

}  // namespace robocallee_fms


namespace robocallee_fms
{

namespace srv
{

namespace builder
{

class Init_DoneMsg_Event_response
{
public:
  explicit Init_DoneMsg_Event_response(::robocallee_fms::srv::DoneMsg_Event & msg)
  : msg_(msg)
  {}
  ::robocallee_fms::srv::DoneMsg_Event response(::robocallee_fms::srv::DoneMsg_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robocallee_fms::srv::DoneMsg_Event msg_;
};

class Init_DoneMsg_Event_request
{
public:
  explicit Init_DoneMsg_Event_request(::robocallee_fms::srv::DoneMsg_Event & msg)
  : msg_(msg)
  {}
  Init_DoneMsg_Event_response request(::robocallee_fms::srv::DoneMsg_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_DoneMsg_Event_response(msg_);
  }

private:
  ::robocallee_fms::srv::DoneMsg_Event msg_;
};

class Init_DoneMsg_Event_info
{
public:
  Init_DoneMsg_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DoneMsg_Event_request info(::robocallee_fms::srv::DoneMsg_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_DoneMsg_Event_request(msg_);
  }

private:
  ::robocallee_fms::srv::DoneMsg_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robocallee_fms::srv::DoneMsg_Event>()
{
  return robocallee_fms::srv::builder::Init_DoneMsg_Event_info();
}

}  // namespace robocallee_fms

#endif  // ROBOCALLEE_FMS__SRV__DETAIL__DONE_MSG__BUILDER_HPP_
